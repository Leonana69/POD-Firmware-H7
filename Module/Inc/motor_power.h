#ifndef __MOTOR_POWER_H__
#define __MOTOR_POWER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "stabilizer_types.h"

#define MOTOR_THRUST_SCALE  1000.0f
#define MOTOR_THRUST_MIN    2000.0f

void motorPowerInit(void);
void motorPowerStop(void);
void motorPowerUpdate(const control_t *control);
bool motorPowerIsFlying();

#ifdef __cplusplus
}
#endif

#endif // __MOTOR_POWER_H__