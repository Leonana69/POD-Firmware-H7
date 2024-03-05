#include "flow.h"
#include "freeRTOS_helper.h"
#include "config.h"
#include "system.h"

#define MODULE_NAME "FLW"
#include "debug.h"

void flowTask(void *argument);

STATIC_TASK_DEF(flowTask, FLOW_TASK_PRIORITY, FLOW_TASK_STACK_SIZE);

void flowInit(void) {
    STATIC_TASK_INIT(flowTask, NULL);
}

void flowTask(void *argument) {
    systemWaitStart();
    DEBUG_PRINT("[START]\n");
    for (;;) {
        osDelay(1000);
    }
}