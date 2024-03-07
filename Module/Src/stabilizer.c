#define MODULE_NAME "STA"
#include "debug.h"

#include "stabilizer.h"
#include "config.h"
#include "freeRTOS_helper.h"
#include "motor_power.h"
#include "system.h"
#include "stabilizer_types.h"

STATIC_TASK_DEF(stabilizerTask, STABILIZER_TASK_PRIORITY, STABILIZER_TASK_STACK_SIZE);

uint32_t stabilizerInit(void) {
    STATIC_TASK_INIT(stabilizerTask, NULL);
    return TASK_INIT_SUCCESS;
}

void stabilizerTask(void *argument) {
    control_t control;
    systemWaitStart();
    while (1) {
        osDelay(100);
    }
}