#ifndef __MOTOR_POWER_H__
#define __MOTOR_POWER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "stabilizer_types.h"

void motorPowerInit(void);
void motorPowerStop(void);
void motorPowerUpdate(const control_t *control);
bool motorPowerIsFlying();

#ifdef __cplusplus
}
#endif

#endif // __MOTOR_POWER_H__