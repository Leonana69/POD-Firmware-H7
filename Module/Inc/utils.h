#ifndef __UTILS_H__
#define __UTILS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void sensorsDelayUs(uint32_t period, void *intf_ptr);
void sensorsDelayMs(uint32_t period);
uint16_t sensorsGetMilli();

float degrees(float radians);
float radians(float degrees);
float clamp(float value, float min, float max);
float canonicalize_angle(float value);

uint32_t getTimeUs();
uint32_t getDurationUs(uint32_t start, uint32_t end);

#ifdef __cplusplus
}
#endif

#endif // __UTILS_H__