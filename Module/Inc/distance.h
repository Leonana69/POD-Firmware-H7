#ifndef __DISTANCE_H__
#define __DISTANCE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

uint32_t distanceInit(void);
void distanceAdjustSpeed(float *vx, float *vy);

#ifdef __cplusplus
}
#endif

#endif // __DISTANCE_H__