#include "utils.h"
#include "cmsis_os.h"
#include "_tim.h"
#include "debug.h"

void delayUs(uint32_t period) {
    uint32_t startTick = __HAL_TIM_GET_COUNTER(&DELAY_US_TIM);
    uint32_t endTick = startTick + period;
    // Handle counter overflow (if applicable)
    if (endTick > DELAY_US_TIM_PERIOD) {
        endTick -= DELAY_US_TIM_PERIOD;
        while (__HAL_TIM_GET_COUNTER(&DELAY_US_TIM) >= endTick) {}
    }
    while (__HAL_TIM_GET_COUNTER(&DELAY_US_TIM) < endTick) {}
}

/*! @brief Sensor delay_us function */
void sensorsDelayUs(uint32_t period, void *intf_ptr) {
    if (period < 20) {
        delayUs(period);
        return;
    }
    osDelay((period + 999) / 1000);
}

/*! @brief Sensor delay_ms function */
void sensorsDelayMs(uint32_t period) {
    osDelay(period);
}

uint16_t sensorsGetMilli() {
    return (uint16_t)osKernelGetTickCount();
}
