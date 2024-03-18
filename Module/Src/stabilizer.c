#define MODULE_NAME "STA"
#include "debug.h"

#include "stabilizer.h"
#include "config.h"
#include "freeRTOS_helper.h"
#include "motor_power.h"
#include "motor_dshot.h"
#include "system.h"
#include "stabilizer_types.h"
#include "controller_pid.h"
#include "estimator_kalman.h"

STATIC_TASK_DEF(stabilizerTask, STABILIZER_TASK_PRIORITY, STABILIZER_TASK_STACK_SIZE);

uint32_t stabilizerInit(void) {
    controllerPidInit();
    STATIC_TASK_INIT(stabilizerTask, NULL);
    return TASK_INIT_SUCCESS;
}

void stabilizerTask(void *argument) {
    state_t state;
    systemWaitStart();
    while (1) {
        estimatorKalmanUpdate(&state);
        DEBUG_PRINT("Stabilizer Task [RUN]: %.3f %.3f %.3f\n", state.position.x, state.position.y, state.position.z);
        osDelay(1000);
    }
}