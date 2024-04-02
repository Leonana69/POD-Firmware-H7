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
    .maxEncodedThrust = MOTOR_2040_MAX_THRUST,
    .minEncodedThrust = MOTOR_2040_MIN_THRUST,
    .divisor = 4096,
    .isFlying = false
};
#else
MotorPower_t motorPower = {
    .init = motorDShotInit,
    .send = motorDShotWriteDma,
    .setRatio = motorDShotSetThrust,
    .getRatio = motorDShotGetThrust,
    .baseThrust = 30000,
    .maxEncodedThrust = DSHOT_MAX_THRUST,
    .minEncodedThrust = DSHOT_MIN_THRUST,
    .divisor = INT16_MAX,
    .isFlying = false
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

static int16_t thrustToRatio(float thrust) {
    int32_t value = thrust * motorPower.maxEncodedThrust / motorPower.divisor;
    value = clamp_i32(value, motorPower.minEncodedThrust, motorPower.maxEncodedThrust);
    return (int16_t) value;
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

#ifdef GEAR
    motorPower.isFlying = false;
#else
    if (t > motorPower.minEncodedThrust) {
        motorPower.isFlying = true;
    } else {
        motorPower.isFlying = false;
    }
#endif
}

void motorPowerSend() {
    motorPower.send();
}

bool motorPowerIsFlying() {
    return motorPower.isFlying;
}