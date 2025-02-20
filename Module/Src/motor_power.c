#include "motor_power.h"
#include "motor_dshot.h"
#include "motor_2040.h"
#include "_config.h"
#include "debug.h"
#include "utils.h"

#ifdef GEAR
MotorPower_t motorPower = {
    .init = motor2040Init,
    .send = motor2040Send,
    .setRatio = motor2040SetSpeed,
    .getRatio = motor2040GetSpeed,
    .baseThrust = 0,
    .maxThrust = MOTOR_2040_MAX_THRUST,
    .minThrust = MOTOR_2040_MIN_THRUST,
    .maxEncodedThrust = MOTOR_2040_MAX_THRUST,
    .minEncodedThrust = MOTOR_2040_MIN_THRUST,
    .isFlying = false,
    .allowNegativeThrust = true
};
#else
MotorPower_t motorPower = {
    .init = motorDShotInit,
    .send = motorDShotWriteDma,
    .setRatio = motorDShotSetThrust,
    .getRatio = motorDShotGetThrust,
    .baseThrust = MOTOR_DSHOT_BASE_THRUST,
    .maxThrust = MOTOR_DSHOT_MAX_VALUE,
    .minThrust = MOTOR_DSHOT_MIN_VALUE,
    .maxEncodedThrust = MOTOR_DSHOT_MAX_THRUST,
    .minEncodedThrust = MOTOR_DSHOT_MIN_THRUST,
    .isFlying = false,
    .allowNegativeThrust = false
};
#endif

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

int16_t motorPowerGetMinThrust() {
    return motorPower.minThrust;
}

int16_t motorPowerGetMaxThrust() {
    return motorPower.maxThrust;
}

int16_t motorPowerGetBaseThrust() {
    return motorPower.baseThrust;
}

static int16_t thrustToRatio(float thrust) {
    int32_t value = thrust * motorPower.maxEncodedThrust / motorPower.maxThrust;

    if (motorPower.allowNegativeThrust) {
        value = clamp_i32(value, -motorPower.maxEncodedThrust, motorPower.maxEncodedThrust);
    } else {
        value = clamp_i32(value, 0, motorPower.maxEncodedThrust);
    }
    
    return (int16_t) value;
}

void motorPowerUpdate(const control_t *control) {
    float r = control->attitude.roll / 2.0f;
    float p = control->attitude.pitch / 2.0f;
    float y = control->attitude.yaw;
    float t = control->thrust;
    int16_t m0 = thrustToRatio(t - r - p - y);
    int16_t m1 = thrustToRatio(t - r + p + y);
    int16_t m2 = thrustToRatio(t + r + p - y);
    int16_t m3 = thrustToRatio(t + r - p + y);
    motorPower.setRatio(0, m0);
    motorPower.setRatio(1, m1);
    motorPower.setRatio(2, m2);
    motorPower.setRatio(3, m3);
    int32_t sum = (ABS(m0) + ABS(m1) + ABS(m2) + ABS(m3)) / 4;
    if (sum > motorPower.minEncodedThrust) {
        motorPower.isFlying = true;
    } else {
        motorPower.isFlying = false;
    }
}

void motorPowerSend() {
    motorPower.send();
}

bool motorPowerIsFlying() {
    return motorPower.isFlying;
}