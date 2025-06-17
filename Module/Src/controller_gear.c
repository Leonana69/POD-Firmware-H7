#include "controller_gear.h"
#include "pid.h"
#include "utils.h"
#include "arm_math.h"
#include "motor_power.h"
#include "supervisor.h"
#include "debug.h"

static pid_t gear_pid_x;
static pid_t gear_pid_y;
static pid_t gear_pid_r;
#define CUTOFF_FREQ 10.0f

void controllerGearInit() {
    pidInit(&gear_pid_x, 20.0f, 0.0f, 0.0f, POSITION_RATE, CUTOFF_FREQ, 0.3f, 4.0f);
    pidInit(&gear_pid_y, 20.0f, 0.0f, 0.0f, POSITION_RATE, CUTOFF_FREQ, 0.3f, 4.0f);
    pidInit(&gear_pid_r, 10.0f, 2.0f, 0.5f, POSITION_RATE, CUTOFF_FREQ, 0.3f, 4.0f);
}

void controllerGearUpdate(setpoint_t *setpoint, imu_t *imu, state_t *state, uint32_t tick, control_t *control_out) {
    static control_t control;
    if (RATE_DO_EXECUTE(POSITION_RATE, tick)) {
        float cos_yaw = cosf(radians(state->attitude.yaw));
        float sin_yaw = sinf(radians(state->attitude.yaw));

        float vx, vy, vr;

        if (setpoint->mode.x == STABILIZE_ABSOLUTE) {
            vx = pidUpdate(&gear_pid_x, setpoint->position.x - state->position.x);
        } else if (setpoint->velocity_body) {
            vx = setpoint->velocity.x * cos_yaw - setpoint->velocity.y * sin_yaw;
        } else {
            vx = setpoint->velocity.x;
        }

        if (setpoint->mode.y == STABILIZE_ABSOLUTE) {
            vy = pidUpdate(&gear_pid_y, setpoint->position.y - state->position.y);
        } else if (setpoint->velocity_body) {
            vy = setpoint->velocity.x * sin_yaw + setpoint->velocity.y * cos_yaw;
        } else {
            vy = setpoint->velocity.y;
        }

        if (setpoint->mode.yaw == STABILIZE_ABSOLUTE) {
            vr = pidUpdate(&gear_pid_r, radians(canonicalize_angle(setpoint->attitude.yaw - state->attitude.yaw)));
        } else if (setpoint->mode.yaw == STABILIZE_VELOCITY) {
            vr = setpoint->palstance.yaw;
        } else {
            vr = 0;
        }

        float target_vx = vx * cos_yaw + vy * sin_yaw;
        float target_vy = -vx * sin_yaw + vy * cos_yaw;

        control.attitude.roll = -vr * MOTOR_THRUST_SCALE;
        control.attitude.pitch = 0;
        control.attitude.yaw = target_vy * MOTOR_THRUST_SCALE;
        control.thrust = target_vx * MOTOR_THRUST_SCALE;
    }

    if ((fabs(control.attitude.roll) < motorPowerGetMinThrust()
    && fabs(control.attitude.yaw) < motorPowerGetMinThrust()
    && fabs(control.thrust) < motorPowerGetMinThrust())
    || supervisorCommandTimeout()) {
        pidReset(&gear_pid_x);
        pidReset(&gear_pid_y);
        pidReset(&gear_pid_r);
        control.attitude.roll = 0;
        control.attitude.pitch = 0;
        control.attitude.yaw = 0;
        control.thrust = 0;
    }

    *control_out = control;
}