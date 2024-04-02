#ifndef __CONTROLLER_GEAR_H__
#define __CONTROLLER_GEAR_H__

#include "stabilizer_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void controllerGearInit();
void controllerGearUpdate(
    setpoint_t *setpoint, imu_t *imu, state_t *state,
    uint32_t tick, control_t *control_out);

#ifdef __cplusplus
}
#endif

#endif // __CONTROLLER_GEAR_H__