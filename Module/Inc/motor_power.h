#ifndef __MOTOR_POWER_H__
#define __MOTOR_POWER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "stabilizer_types.h"

#define MOTOR_THRUST_SCALE  500.0f
#define MOTOR_THRUST_BASE   15000.0f
#define MOTOR_THRUST_MIN    1000.0f
#define MOTOR_THRUST_MAX    32000.0f

typedef struct {
    void (*init)();
    void (*send)();
    void (*setRatio)(uint8_t id, int16_t thrust);
    int16_t (*getRatio)(uint8_t id);
    int16_t baseThrust;
    int16_t maxThrust;
    int16_t minThrust;
    int16_t maxEncodedThrust;
    int16_t minEncodedThrust;
    bool isFlying;
    bool allowNegativeThrust;
} MotorPower_t;

void motorPowerInit();
void motorPowerStop();
void motorPowerSend();
int16_t motorPowerGetMinThrust();
void motorPowerUpdate(const control_t *control);
bool motorPowerIsFlying();

#ifdef __cplusplus
}
#endif

#endif // __MOTOR_POWER_H__