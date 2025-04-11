#ifndef __CONTROLLER_DRONE_H__
#define __CONTROLLER_DRONE_H__

#include "stabilizer_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void controllerDroneInit();
void controllerDroneUpdate(
    setpoint_t *setpoint, imu_t *imu, state_t *state,
    uint32_t tick, control_t *control_out);
void controllerDroneEnableObstacleAvoidance(bool enable);

#ifdef __cplusplus
}
#endif

#endif // __CONTROLLER_DRONE_H__