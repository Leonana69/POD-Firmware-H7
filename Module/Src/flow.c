#define MODULE_NAME "FLW"
#include "debug.h"

#include "flow.h"
#include "freeRTOS_helper.h"
#include "config.h"
#include "system.h"

STATIC_TASK_DEF(flowTask, FLOW_TASK_PRIORITY, FLOW_TASK_STACK_SIZE);

uint32_t flowInit(void) {
    STATIC_TASK_INIT(flowTask, NULL);
    DEBUG_PRINT("Flow Init [OK]\n");
    return TASK_INIT_SUCCESS;
}

void flowTask(void *argument) {
    systemWaitStart();
    
    for (;;) {
        osDelay(1000);
        DEBUG_PRINT("[RUN]\n");
    }
}