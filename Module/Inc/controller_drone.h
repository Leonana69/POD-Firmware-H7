#ifndef __CONTROLLER_DRONE_H__
#define __CONTROLLER_DRONE_H__

#include "stabilizer_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CONTROLLER_OA_NONE = 0x0,
    CONTROLLER_OA_STOP = 0x1,
    CONTROLLER_OA_AVOID = 0x2,
} controller_oa_t;

void controllerDroneInit();
void controllerDroneUpdate(
    setpoint_t *setpoint, imu_t *imu, state_t *state,
    uint32_t tick, control_t *control_out);
void controllerDroneSetOA(controller_oa_t mode);

#ifdef __cplusplus
}
#endif

#endif // __CONTROLLER_DRONE_H__