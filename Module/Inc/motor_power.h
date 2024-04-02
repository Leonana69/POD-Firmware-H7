#ifndef __MOTOR_POWER_H__
#define __MOTOR_POWER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "stabilizer_types.h"

#define MOTOR_THRUST_SCALE  1000.0f
#define MOTOR_THRUST_BASE   30000.0f
#define MOTOR_THRUST_MIN    2000.0f
#define MOTOR_THRUST_MAX    60000.0f

typedef struct {
    void (*init)();
    void (*send)();
    void (*setRatio)(uint8_t id, uint16_t thrust);
    uint16_t (*getRatio)(uint8_t id);
    uint16_t baseThrust;
    uint16_t maxEncodedThrust;
    uint16_t minEncodedThrust;
    bool isFlying;
} MotorPower_t;

void motorPowerInit();
void motorPowerStop();
void motorPowerSend();
void motorPowerUpdate(const control_t *control);
bool motorPowerIsFlying();

#ifdef __cplusplus
}
#endif

#endif // __MOTOR_POWER_H__