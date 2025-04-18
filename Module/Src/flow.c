#define MODULE_NAME "FLW"
#include "debug.h"

#include "flow.h"
#include "task_config.h"
#include "system.h"
#include "paa3905.h"
#include "utils.h"
#include "_spi.h"
#include "_config.h"
#include "stabilizer_types.h"
#include "estimator_kalman.h"

STATIC_TASK_DEF(flowTask, FLOW_TASK_PRIORITY, FLOW_TASK_STACK_SIZE);

#define FLOW_TASK_RATE RATE_100_HZ
#define FLOW_STD_DEV 0.2f
static paa3905_dev_t paa3905_dev;

uint32_t flowInit(void) {
#ifdef GEAR
    // Gear does not use flow
    DEBUG_PRINT("Flow Init [SKIPPED]\n");
    return TASK_INIT_SUCCESS;
#endif
    paa3905_dev.delay = sensorsDelayMs;
    paa3905_dev.read = paa3905_read_dma;
    paa3905_dev.write = paa3905_write_dma;
    paa3905_dev.mode = PAA3905_STANDARD_MODE;

    uint8_t rslt = paa3905_init(&paa3905_dev);
    paa3905_disable_motion_cutoff(&paa3905_dev);

    if (rslt != PAA3905_OK) {
        DEBUG_PRINT("PAA3905 Init [FAILED]: rslt: %d\n", rslt);
        return TASK_INIT_FAILED(FLOW_TASK_INDEX);
    } else {
        DEBUG_PRINT("PAA3905 Init [OK]\n");
        STATIC_TASK_INIT(flowTask, NULL);
    }
    return TASK_INIT_SUCCESS;
}

uint8_t squalThreshold[3] = { 0x19, 0x46, 0x55 };
uint32_t shutterThreshold[3] = { 0x00FF80, 0x00FF80, 0x025998 };
void flowTask(void *argument) {
    estimatorPacket_t packet = { .type = FLOW_TASK_INDEX };
    paa3905_motion_t motion;
    uint32_t lastTime = getTimeUs();
    systemWaitStart();
    TASK_TIMER_DEF(FLOW, FLOW_TASK_RATE);
    while (1) {
        TASK_TIMER_WAIT(FLOW);
        paa3905_motion_burst(&paa3905_dev, &motion);

        int mode = (motion.observation >> 6) & 0x03;
        uint32_t shutter = motion.shutter_lower | (motion.shutter_middle << 8) | (motion.shutter_upper << 16);
        if ((motion.squal < squalThreshold[mode] && shutter >= shutterThreshold[mode]) || (motion.motion & 0x01)) {
            packet.flow.dpixelx = 0;
            packet.flow.dpixely = 0;
            continue;
        } else {
            packet.flow.dpixelx = -motion.delta_y;
            packet.flow.dpixely = motion.delta_x;
        }

        packet.flow.stdDevX = FLOW_STD_DEV;
        packet.flow.stdDevY = FLOW_STD_DEV;
        uint32_t currentTime = getTimeUs();
        packet.flow.dt = getDurationUs(lastTime, currentTime) / 1e6f;
        lastTime = currentTime;

        // DEBUG_REMOTE("Flow: %d, %d, %d\n", motion.delta_x, motion.delta_y, motion.challenging_condition);
        estimatorKalmanEnqueue(&packet);
    }
}