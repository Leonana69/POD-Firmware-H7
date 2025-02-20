#define MODULE_NAME "STA"
#include "debug.h"

#include <string.h>

#include "stabilizer.h"
#include "config.h"
#include "freeRTOS_helper.h"
#include "motor_power.h"
#include "motor_dshot.h"
#include "system.h"
#include "stabilizer_types.h"
#include "controller.h"
#include "estimator_kalman.h"
#include "imu.h"
#include "supervisor.h"
#include "led_seq.h"
#include "command.h"

STATIC_TASK_DEF(stabilizerTask, STABILIZER_TASK_PRIORITY, STABILIZER_TASK_STACK_SIZE);

uint32_t stabilizerInit(void) {
    controllerInit();
    STATIC_TASK_INIT(stabilizerTask, NULL);
    return TASK_INIT_SUCCESS;
}

void compressState(state_com_t *s, state_t *state) {
    s->x = state->position.x * 1000;
    s->y = state->position.y * 1000;
    s->z = state->position.z * 1000;
    float const deg2millirad = ((float)M_PI * 1000.0f) / 180.0f;
    s->roll = state->attitude.roll * deg2millirad;
    s->pitch = state->attitude.pitch * deg2millirad;
    s->yaw = state->attitude.yaw * deg2millirad;
    s->timestamp = osKernelGetTickCount();
}

extern bool isTumbled;
extern bool isLocked;
void stabilizerTask(void *argument) {
    state_t state;
    state_com_t stateCom;
    control_t control;
    setpoint_t setpoint;
    imu_t imu;
    systemWaitStart();
    LED_SEQ_CALL(TakeOff);
    uint32_t tick = 0;
    while (1) {
        imuWaitData();
        imuGetData(&imu);

        supervisorUpdate(&imu);

        estimatorKalmanUpdate(&state);
        compressState(&stateCom, &state);

        // if (tick % 100 == 0) {
        //     linkSendData(PODTP_TYPE_LOG, PORT_LOG_STATE, (uint8_t *) &stateCom, sizeof(stateCom));
        // }

        // if (tick % 250 == 0) {
        //     DEBUG_REMOTE("(%.2f, %.2f, %.2f), (%.1f, %.1f, %.1f)\n", state.position.x, state.position.y, state.position.z, 
        //     state.attitude.roll, state.attitude.pitch, state.attitude.yaw);
        // }
        // if (tick % 500 == 0) {
        //     DEBUG_REMOTE("%.2f,\t%.2f,\t%.2f;\t%.1f,\t%.1f,\t%.1f;\t%.1f,\t%.1f,\t%.1f\n", state.position.x, state.position.y, state.position.z, 
        //     state.velocity.x, state.velocity.y, state.velocity.z,
        //     state.attitude.roll, state.attitude.pitch, state.attitude.yaw);
        // }

        // if (tick % 250 == 0) {
        //     DEBUG_REMOTE("%.2f,\t%.2f,\t%.2f;    %.2f,\t%.2f,\t%.2f\n", 
        //         imu.accel.x, imu.accel.y, imu.accel.z,
        //         imu.gyro.x, imu.gyro.y, imu.gyro.z);
        // }

        commandGetSetpoint(&setpoint);

        controllerUpdate(&setpoint, &imu, &state, tick, &control);
        
        if (supervisorCanFly()) {
            motorPowerUpdate(&control);
        } else {
            motorPowerStop();
        }

        tick++;
    }
}