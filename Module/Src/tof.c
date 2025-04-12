#define MODULE_NAME "TOF"
#include "debug.h"

#include "tof.h"
#include "_i2c.h"
#include "_config.h"
#include "task_config.h"
#include "vl53l1.h"
#include "utils.h"
#include "system.h"
#include "stabilizer_types.h"
#include "estimator_kalman.h"

#define expCoeff 2.92135f
#define expPointA 2.5f
#define expStdA 0.0025f
#define RANGE_MAX 5000
#define RANGE_MIN 10

STATIC_TASK_DEF(tofTask, TOF_TASK_PRIORITY, TOF_TASK_STACK_SIZE);

#define TOF_TASK_RATE RATE_50_HZ
static VL53L1_Dev_t vl53l1Dev;

uint32_t tofInit(void) {
    if (HAL_I2C_IsDeviceReady(&VL53L1_I2C_HANDLE, VL53L1_DEV_ADDR_DEFAULT << 1, 3, 100) != HAL_OK) {
        DEBUG_PRINT("VL53L1 not found\n");
        return TASK_INIT_FAILED(TOF_TASK_INDEX);
    }

    vl53l1Dev.read = vl53l1_read_dma;
    vl53l1Dev.write = vl53l1_write_normal;
    vl53l1Dev.delay = sensorsDelayMs;
    vl53l1Dev.millis = sensorsGetMilli;
    vl53l1Dev.i2c_slave_address = VL53L1_DEV_ADDR_DEFAULT;

    int8_t status = vl53l1Init(&vl53l1Dev);
    if (status != VL53L1_ERROR_NONE) {
        DEBUG_PRINT("VL53L1 Init [FAILED]: %d\n", status);
        return TASK_INIT_FAILED(TOF_TASK_INDEX);
    } else {
        DEBUG_PRINT("VL53L1 Init [OK]\n");
        STATIC_TASK_INIT(tofTask, NULL);
    }

    return TASK_INIT_SUCCESS;
}

void tofTask(void *argument) {
    estimatorPacket_t packet = { .type = TOF_TASK_INDEX };
    VL53L1_RangingMeasurementData_t vl53l1RangingData;
    systemWaitStart();
    uint32_t lastTime = getTimeUs();
    TASK_TIMER_DEF(TOF, TOF_TASK_RATE);
	while (1) {
		TASK_TIMER_WAIT(TOF);
		VL53L1_WaitMeasurementDataReady(&vl53l1Dev);
		VL53L1_GetRangingMeasurementData(&vl53l1Dev, &vl53l1RangingData);
		int16_t range_mm = vl53l1RangingData.RangeMilliMeter;
		VL53L1_ClearInterruptAndStartMeasurement(&vl53l1Dev);

		if (range_mm < RANGE_MAX) {
            if (range_mm < RANGE_MIN) {
                range_mm = RANGE_MIN;
            }
			packet.tof.distance = range_mm * 0.001f;
			packet.tof.stdDev = expStdA * (1.0f  + expf(expCoeff * (range_mm * 0.001f - expPointA)));
            uint32_t currentTime = getTimeUs();
            packet.tof.dt = getDurationUs(lastTime, currentTime) / 1e6f;
            lastTime = currentTime;
            estimatorKalmanEnqueue(&packet);
		}
	}
}