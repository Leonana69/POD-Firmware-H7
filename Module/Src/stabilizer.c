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
#include "controller_pid.h"
#include "estimator_kalman.h"
#include "imu.h"
#include "supervisor.h"
#include "led_seq.h"
#include "command.h"

STATIC_TASK_DEF(stabilizerTask, STABILIZER_TASK_PRIORITY, STABILIZER_TASK_STACK_SIZE);

uint32_t stabilizerInit(void) {
    controllerPidInit();
    STATIC_TASK_INIT(stabilizerTask, NULL);
    return TASK_INIT_SUCCESS;
}
extern MotorPower_t motorPower;
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

        controllerPidUpdate(&setpoint, &imu, &state, tick, &control);

        if (tick % 1000 == 0) {
            DEBUG_PRINT("c: %.1f %.1f %.1f %.1f\n", control.attitude.roll, control.attitude.pitch, control.attitude.yaw, control.thrust);
        }
        
        if (supervisorCanFly()) {
            motorPowerUpdate(&control);
        } else {
            supervisorLockDrone(true);
            motorPowerStop();
        }

        if (tick % 1000 == 0)
            DEBUG_PRINT("s: %.3f %.3f %.3f\n", state.position.x, state.position.y, state.position.z);
        
        tick++;
    }
}