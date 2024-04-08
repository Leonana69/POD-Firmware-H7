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

void stabilizerTask(void *argument) {
    state_t state;
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

        if (supervisorCommandTimeout())
            memset(&setpoint, 0, sizeof(setpoint_t));
        else {
            commandGetSetpoint(&setpoint);
        }

        controllerUpdate(&setpoint, &imu, &state, tick, &control);
        
        if (supervisorCanFly()) {
            motorPowerUpdate(&control);
        } else {
            supervisorLockDrone(true);
            motorPowerStop();
        }

        tick++;
    }
}