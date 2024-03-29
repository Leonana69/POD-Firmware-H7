#include "motor_power.h"
#include "motor_dshot.h"
#include "_config.h"
#include "debug.h"

MotorPower_t motorPower = {
    .init = motorDShotInit,
    .setRatio = motorDShotSetThrust,
    .getRatio = motorDShotGetThrust,
    .baseThrust = 30000,
    .maxThrust = MOTOR_THRUST_MAX,
    .minThrust = MOTOR_THRUST_MIN,
    .isFlying = false
};

void motorPowerInit(void) {
    motorPower.init();
    motorPowerStop();
}

void motorPowerStop(void) {
    for (int i = 0; i < MOTOR_COUNT; i++) {
        motorPower.setRatio(i, 0);
    }
    motorPower.isFlying = false;
}

static uint16_t thrustToRatio(uint16_t thrust) {
    uint32_t value = thrust * motorPower.maxThrust / UINT16_MAX;
    return value & 0xFFFF;
}

void motorPowerUpdate(const control_t *control) {
    float r = control->attitude.roll / 2.0f;
    float p = control->attitude.pitch / 2.0f;
    float y = control->attitude.yaw;
    float t = control->thrust;
    motorPower.setRatio(0, thrustToRatio(t - r + p + y));
    motorPower.setRatio(1, thrustToRatio(t - r - p - y));
    motorPower.setRatio(2, thrustToRatio(t + r - p + y));
    motorPower.setRatio(3, thrustToRatio(t + r + p - y));

    if (t > motorPower.minThrust) {
        motorPower.isFlying = true;
    } else {
        motorPower.isFlying = false;
    }
}

bool motorPowerIsFlying() {
    return motorPower.isFlying;
}