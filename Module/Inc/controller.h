#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

#include "stabilizer_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void controllerInit();
void controllerUpdate(
    setpoint_t *setpoint, imu_t *imu, state_t *state,
    uint32_t tick, control_t *control_out);

#ifdef __cplusplus
}
#endif

#endif // __CONTROLLER_H__