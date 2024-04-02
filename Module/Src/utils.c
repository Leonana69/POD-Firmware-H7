#include "utils.h"
#include "cmsis_os.h"
#include "_tim.h"
#include "debug.h"

uint32_t getTimeUs() {
    return __HAL_TIM_GET_COUNTER(&DELAY_US_TIM);
}

uint32_t getDurationUs(uint32_t start, uint32_t end) {
    if (end >= start) {
        return end - start;
    } else {
        return DELAY_US_TIM_PERIOD - start + end;
    }
}

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

float degrees(float radians) {
    return radians * 57.2957795130823208767981548141051703;
}

float radians(float degrees) {
    return degrees * 0.0174532925199432957692369076848861;
}

float clamp_f(float value, float min, float max) {
    if (value < min) {
        return min;
    } else if (value > max) {
        return max;
    } else {
        return value;
    }
}

int32_t clamp_i32(int32_t value, int32_t min, int32_t max) {
    if (value < min) {
        return min;
    } else if (value > max) {
        return max;
    } else {
        return value;
    }
}

float canonicalize_angle(float value) {
    while (value > 180.0f) {
        value -= 360.0f;
    }
    while (value < -180.0f) {
        value += 360.0f;
    }
    return value;
}