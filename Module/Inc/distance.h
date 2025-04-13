#ifndef __DISTANCE_H__
#define __DISTANCE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stabilizer_types.h"
#include "controller_drone.h"

uint32_t distanceInit(void);
void distanceAdjustSpeed(state_t *state, float *vx, float *vy, controller_oa_t mode);

#ifdef __cplusplus
}
#endif

#endif // __DISTANCE_H__