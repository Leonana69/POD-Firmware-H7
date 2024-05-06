#include "controller_drone.h"
#include "pid.h"
#include "utils.h"
#include "motor_power.h"
#include "arm_math.h"

static pid_t pid_roll, pid_pitch, pid_yaw;
static pid_t pid_roll_rate, pid_pitch_rate, pid_yaw_rate;

#define OUTTER_LOOP_CUTOFF_FREQ 10.0f
#define INNER_LOOP_CUTOFF_FREQ 100.0f

/*
 * kp = kap + wo * kad, ki = wo * kap, kd = kad + wo
 */

void controllerPidAttitudeInit(void) {
    pidInit(&pid_roll, 5.0f, 2.0f, 0.0f, ATTITUDE_RATE, OUTTER_LOOP_CUTOFF_FREQ, 20.0f, 0.0f);
    pidInit(&pid_pitch, 5.0f, 2.0f, 0.0f, ATTITUDE_RATE, OUTTER_LOOP_CUTOFF_FREQ, 20.0f, 0.0f);
    pidInit(&pid_yaw, 6.0f, 1.0f, 0.35f, ATTITUDE_RATE, OUTTER_LOOP_CUTOFF_FREQ, 360.0f, 0.0f);

    pidInit(&pid_roll_rate, 150.0f, 200.0f, 1.0f, ATTITUDE_RATE, INNER_LOOP_CUTOFF_FREQ, 33.0f, 0.0f);
    pidInit(&pid_pitch_rate, 150.0f, 200.0f, 1.0f, ATTITUDE_RATE, INNER_LOOP_CUTOFF_FREQ, 33.0f, 0.0f);
    pidInit(&pid_yaw_rate, 100.0f, 20.0f, 0.0f, ATTITUDE_RATE, INNER_LOOP_CUTOFF_FREQ, 33.0f, 0.0f);

    // kap = 5.0f, kad = 2.0f, wo = 20.0f
    // pidInit(&pid_roll_rate, 45, 100, 22, ATTITUDE_RATE, INNER_LOOP_CUTOFF_FREQ, 33.0f, 0.0f);
    // pidInit(&pid_pitch_rate, 45, 100, 22, ATTITUDE_RATE, INNER_LOOP_CUTOFF_FREQ, 33.0f, 0.0f);
    // pidInit(&pid_yaw_rate, 45, 100, 22, ATTITUDE_RATE, INNER_LOOP_CUTOFF_FREQ, 33.0f, 0.0f);
}

void controllerPidAttitudeUpdateValue(attitude_t *estimation, attitude_t *target, palstance_t *palstance) {
    palstance->roll = pidUpdate(&pid_roll, target->roll - estimation->roll);
    palstance->pitch = pidUpdate(&pid_pitch, target->pitch - estimation->pitch);
    palstance->yaw = pidUpdate(&pid_yaw, canonicalize_angle(target->yaw - estimation->yaw));
}

void controllerPidAttitudeUpdateRate(palstance_t *measurement, palstance_t *target, control_t *control) {
    control->attitude.roll = pidUpdate(&pid_roll_rate, target->roll - measurement->roll);
    control->attitude.pitch = pidUpdate(&pid_pitch_rate, target->pitch - measurement->pitch);
    control->attitude.yaw = pidUpdate(&pid_yaw_rate, target->yaw - measurement->yaw);
}

void controllerPidAttitudeUpdate(
    setpoint_t *setpoint, imu_t *imu, state_t *state,
    attitude_t *attitude_target, palstance_t *palstance_target,
    scalar_t *thrust, control_t *control) {

    if (setpoint->mode.yaw == STABILIZE_VELOCITY) {
        attitude_target->yaw += setpoint->palstance.yaw * (1.0f / ATTITUDE_RATE);
    } else if (setpoint->mode.yaw == STABILIZE_ABSOLUTE) {
        attitude_target->yaw = setpoint->attitude.yaw;
    }
    attitude_target->yaw = canonicalize_angle(attitude_target->yaw);

    // no control for z axis, set thrust directly
    if (setpoint->mode.z == STABILIZE_DISABLE) {
        *thrust = setpoint->thrust;
    }

    if (setpoint->mode.x == STABILIZE_DISABLE || setpoint->mode.y == STABILIZE_DISABLE) {
        attitude_target->roll = setpoint->attitude.roll;
        attitude_target->pitch = setpoint->attitude.pitch;
    }

    controllerPidAttitudeUpdateValue(&state->attitude, attitude_target, palstance_target);

    if (setpoint->mode.roll == STABILIZE_VELOCITY) {
        palstance_target->roll = setpoint->palstance.roll;
        pidReset(&pid_roll);
    }

    if (setpoint->mode.pitch == STABILIZE_VELOCITY) {
        palstance_target->pitch = setpoint->palstance.pitch;
        pidReset(&pid_pitch);
    }

    palstance_t measurement = {
        .roll = imu->gyro.x,
        .pitch = -imu->gyro.y,
        .yaw = imu->gyro.z
    };
    controllerPidAttitudeUpdateRate(&measurement, palstance_target, control);
    control->thrust = *thrust;
}

void controllerPidAttitudeReset(void) {
    pidReset(&pid_roll);
    pidReset(&pid_pitch);
    pidReset(&pid_yaw);
    pidReset(&pid_roll_rate);
    pidReset(&pid_pitch_rate);
    pidReset(&pid_yaw_rate);
}

static pid_t pid_x, pid_y, pid_z;
static pid_t pid_x_rate, pid_y_rate, pid_z_rate;

void controllerPidPositionInit(void) {
    pidInit(&pid_x, 2.0f, 0.5f, 0.0f, POSITION_RATE, OUTTER_LOOP_CUTOFF_FREQ, 1.0f, 0.0f);
    pidInit(&pid_y, 2.0f, 0.5f, 0.0f, POSITION_RATE, OUTTER_LOOP_CUTOFF_FREQ, 1.0f, 0.0f);
    pidInit(&pid_z, 2.0f, 0.5f, 0.0f, POSITION_RATE, OUTTER_LOOP_CUTOFF_FREQ, 1.0f, 0.0f);

    pidInit(&pid_x_rate, 25.0f, 2.0f, 0.0f, POSITION_RATE, INNER_LOOP_CUTOFF_FREQ, 25.0f, 20.0f);
    pidInit(&pid_y_rate, 25.0f, 2.0f, 0.0f, POSITION_RATE, INNER_LOOP_CUTOFF_FREQ, 25.0f, 20.0f);
    pidInit(&pid_z_rate, 15.0f, 10.0f, 1.0f, POSITION_RATE, INNER_LOOP_CUTOFF_FREQ, 20.0f, MOTOR_THRUST_MAX / MOTOR_THRUST_SCALE);
}

void controllerPidPositionUpdate(setpoint_t *setpoint, state_t *state, attitude_t *attitude_target, scalar_t *thrust) {
    float cos_yaw = cosf(radians(state->attitude.yaw));
    float sin_yaw = sinf(radians(state->attitude.yaw));

    float vx, vy, vz;

    if (setpoint->mode.x == STABILIZE_ABSOLUTE) {
        vx = pidUpdate(&pid_x, setpoint->position.x - state->position.x);
    } else if (setpoint->velocity_body) {
        vx = setpoint->velocity.x * cos_yaw - setpoint->velocity.y * sin_yaw;
    } else {
        vx = setpoint->velocity.x;
    }

    if (setpoint->mode.y == STABILIZE_ABSOLUTE) {
        vy = pidUpdate(&pid_y, setpoint->position.y - state->position.y);
    } else if (setpoint->velocity_body) {
        vy = setpoint->velocity.x * sin_yaw + setpoint->velocity.y * cos_yaw;
    } else {
        vy = setpoint->velocity.y;
    }

    if (setpoint->mode.z == STABILIZE_ABSOLUTE) {
        vz = pidUpdate(&pid_z, setpoint->position.z - state->position.z);
    } else {
        vz = setpoint->velocity.z;
    }

    float ax = pidUpdate(&pid_x_rate, vx - state->velocity.x);
    float ay = pidUpdate(&pid_y_rate, vy - state->velocity.y);
    float az = pidUpdate(&pid_z_rate, vz - state->velocity.z);

    attitude_target->roll = ax * sin_yaw - ay * cos_yaw;
    attitude_target->pitch = -(ax * cos_yaw + ay * sin_yaw);
    *thrust = az * MOTOR_THRUST_SCALE + motorPowerGetBaseThrust();
}

void controllerPidPositionReset() {
    pidReset(&pid_x);
    pidReset(&pid_y);
    pidReset(&pid_z);
    pidReset(&pid_x_rate);
    pidReset(&pid_y_rate);
    pidReset(&pid_z_rate);
}

void controllerDroneInit() {
    controllerPidAttitudeInit();
    controllerPidPositionInit();
}

void controllerDroneUpdate(setpoint_t *setpoint, imu_t *imu, state_t *state, uint32_t tick, control_t *control_out) {
    static attitude_t attitude_target;
    static palstance_t palstance_target;
    static scalar_t thrust;
    static control_t control;

    if (RATE_DO_EXECUTE(POSITION_RATE, tick)) {
        controllerPidPositionUpdate(setpoint, state, &attitude_target, &thrust);
    }

    if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
        controllerPidAttitudeUpdate(setpoint, imu, state, &attitude_target, &palstance_target, &thrust, &control);
    }

    if (control.thrust < (float) motorPowerGetMinThrust()) {
        controllerPidAttitudeReset();
        controllerPidPositionReset();
        attitude_target.yaw = state->attitude.yaw;
        control.attitude.roll = 0;
        control.attitude.pitch = 0;
        control.attitude.yaw = 0;
        control.thrust = 0;
    }

    *control_out = control;
}