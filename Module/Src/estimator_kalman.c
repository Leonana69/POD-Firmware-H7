#define MODULE_NAME "EST"
#include "debug.h"

#include "estimator_kalman.h"
#include "stabilizer_types.h"
#include "config.h"
#include "kalman_core.h"
#include "kalman_update.h"
#include "freeRTOS_helper.h"
#include "system.h"
#include "utils.h"
#include "motor_power.h"

#define ESTIMATOR_TASK_RATE RATE_1000_HZ
#define ESTIMATOR_PREDICTION_RATE RATE_100_HZ

DATA_REGION static kalmanCoreData_t coreData;
DATA_REGION static state_t stateData;

STATIC_MUTEX_DEF(estimatorDataMutex);
STATIC_QUEUE_DEF(estimatorDataQueue, 30, estimatorPacket_t);
STATIC_TASK_DEF(estimatorKalmanTask, ESTIMATOR_TASK_PRIORITY, ESTIMATOR_TASK_STACK_SIZE);

uint32_t estimatorKalmanInit() {
    kalmanCoreInit(&coreData);
    STATIC_MUTEX_INIT(estimatorDataMutex);
    STATIC_QUEUE_INIT(estimatorDataQueue);
    STATIC_TASK_INIT(estimatorKalmanTask, NULL);
    return TASK_INIT_SUCCESS;
}

void estimatorKalmanUpdate(state_t *state) {
    STATIC_MUTEX_LOCK(estimatorDataMutex, osWaitForever);
    *state = stateData;
    STATIC_MUTEX_UNLOCK(estimatorDataMutex);
}

void estimatorKalmanEnqueue(estimatorPacket_t *packet) {
    STATIC_QUEUE_SEND(estimatorDataQueue, packet, 0);
}

static imu_t imuAccumulator = { 0 };
static imu_t latestImu = { 0 };
static uint32_t imuCount = 0;
static uint32_t lastImuPrediction;
bool processDataQueue() {
    estimatorPacket_t packet;
    bool update = false;
    while (STATIC_QUEUE_RECEIVE(estimatorDataQueue, &packet, 0) == osOK) {
        switch (packet.type) {
            case IMU_TASK_INDEX:
                latestImu = packet.imu;
                for (int i = 0; i < 3; i++) {
                    imuAccumulator.gyro.v[i] += packet.imu.gyro.v[i];
                    imuAccumulator.accel.v[i] += packet.imu.accel.v[i];
                }
                imuCount++;
                break;
            case FLOW_TASK_INDEX:
                kalmanCoreUpdateWithFlow(&coreData, &packet.flow, &latestImu.gyro);
                update = true;
                break;
            case TOF_TASK_INDEX:
                kalmanCoreUpdateWithTof(&coreData, &packet.tof);
                update = true;
                break;
            default:
                break;
        }
    }

    uint32_t dt = getDurationUs(lastImuPrediction, getTimeUs());
    if (imuCount > 0 && dt >= 1000000 / ESTIMATOR_PREDICTION_RATE) {
        for (int i = 0; i < 3; i++) {
            imuAccumulator.gyro.v[i] = radians(imuAccumulator.gyro.v[i] / imuCount);
            imuAccumulator.accel.v[i] = imuAccumulator.accel.v[i] * GRAVITY_EARTH / imuCount;
        }
        kalmanCorePredict(&coreData, &imuAccumulator, dt / 1000000.0f, motorPowerIsFlying());
        imuCount = 0;
        imuAccumulator = (imu_t){ 0 };
        lastImuPrediction = getTimeUs();
        update = true;
    }

    return update;
}

void estimatorKalmanTask(void *argument) {
    systemWaitStart();
    TASK_TIMER_DEF(ESTIMATOR, ESTIMATOR_TASK_RATE);
    uint32_t lastTime = getTimeUs();
    bool update;
    while (1) {
        TASK_TIMER_WAIT(ESTIMATOR);

        // reset kalman core if necessary
        if (coreData.reset) {
            kalmanCoreInit(&coreData);
        }

        update = processDataQueue();

        uint32_t currentTime = getTimeUs();
        kalmanCoreAddProcessNoise(&coreData, getDurationUs(lastTime, currentTime) / 1e6f);
        lastTime = currentTime;
        
        if (update) {
            kalmanCoreFinalize(&coreData);
            if (!kalmanCoreCheckBounds(&coreData)) {
                DEBUG_PRINT("Kalman Core [RESET]: P(%.3f %.3f %.3f), V(%.3f %.3f %.3f)\n",
                    coreData.S[1], coreData.S[0], coreData.S[2],
                    coreData.S[3], coreData.S[4], coreData.S[5]);
                kalmanCoreInit(&coreData);
            }
        }

        STATIC_MUTEX_LOCK(estimatorDataMutex, osWaitForever);
        kalmanCoreExternalizeState(&coreData, &stateData, &latestImu.accel, osKernelGetTickCount());
        STATIC_MUTEX_UNLOCK(estimatorDataMutex);
    }
}