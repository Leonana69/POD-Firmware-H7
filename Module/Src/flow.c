#define MODULE_NAME "FLW"
#include "debug.h"

#include "flow.h"
#include "config.h"
#include "system.h"
#include "paa3905.h"
#include "utils.h"
#include "_spi.h"
#include "_config.h"
#include "stabilizer_types.h"
#include "estimator_kalman.h"

STATIC_TASK_DEF(flowTask, FLOW_TASK_PRIORITY, FLOW_TASK_STACK_SIZE);

#define FLOW_RATE RATE_50_HZ
#define FLOW_STD_DEV 1.0f
static paa3905_dev_t paa3905_dev;

uint32_t flowInit(void) {
    paa3905_dev.delay = sensorsDelayMs;
    paa3905_dev.read = paa3905_read_dma;
    paa3905_dev.write = paa3905_write_dma;
    paa3905_dev.mode = PAA3905_STANDARD_MODE;

    uint8_t rslt = paa3905_init(&paa3905_dev);

    if (rslt != PAA3905_OK) {
        DEBUG_PRINT("PAA3905 Init [FAILED]: rslt: %d\n", rslt);
        return TASK_INIT_FAILED(FLOW_TASK_INDEX);
    } else {
        DEBUG_PRINT("PAA3905 Init [OK]\n");
        STATIC_TASK_INIT(flowTask, NULL);
    }
    return TASK_INIT_SUCCESS;
}

void flowTask(void *argument) {
    estimatorPacket_t packet = { .type = FLOW_TASK_INDEX };
    paa3905_motion_t motion;
    uint32_t lastTime = getTimeUs();
    systemWaitStart();
    TASK_TIMER_DEF(FLOW, FLOW_RATE);
    while (1) {
        TASK_TIMER_WAIT(FLOW);
        paa3905_motion_burst(&paa3905_dev, &motion);
        packet.flow.dpixelx = motion.delta_x;
        packet.flow.dpixely = motion.delta_y;
        packet.flow.stdDevX = FLOW_STD_DEV;
        packet.flow.stdDevY = FLOW_STD_DEV;
        uint32_t currentTime = getTimeUs();
        packet.flow.dt = getDurationUs(lastTime, currentTime) / 1e6f;
        lastTime = currentTime;
        estimatorKalmanEnqueue(&packet);
    }
}