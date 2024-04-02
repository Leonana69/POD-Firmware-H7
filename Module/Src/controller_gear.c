#include "controller_gear.h"
#include "pid.h"
#include "utils.h"
#include "arm_math.h"
#include "motor_power.h"
#include "supervisor.h"

static pid_t pid_x, pid_y, pid_r;
#define CUTOFF_FREQ 10.0f

void controllerGearInit() {
    pidInit(&pid_x, 1.0f, 0.5f, 0.0f, POSITION_RATE, CUTOFF_FREQ, 0.3f, 2.0f);
    pidInit(&pid_y, 1.0f, 0.5f, 0.0f, POSITION_RATE, CUTOFF_FREQ, 0.3f, 2.0f);
    pidInit(&pid_r, 1.0f, 0.5f, 0.0f, POSITION_RATE, CUTOFF_FREQ, 0.3f, 1.0f);
}

void controllerGearUpdate(setpoint_t *setpoint, imu_t *imu, state_t *state, uint32_t tick, control_t *control_out) {
    static scalar_t yaw;
    static control_t control;
    if (RATE_DO_EXECUTE(POSITION_RATE, tick)) {
        float cos_yaw = cosf(radians(state->attitude.yaw));
        float sin_yaw = sinf(radians(state->attitude.yaw));

        float vx, vy, vr;
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

        if (setpoint->mode.yaw == STABILIZE_VELOCITY) {
            yaw += setpoint->palstance.yaw * (1.0f / POSITION_RATE);
        } else if (setpoint->mode.yaw == STABILIZE_ABSOLUTE) {
            yaw = setpoint->attitude.yaw;
        }
        yaw = canonicalize_angle(yaw);

        vr = pidUpdate(&pid_r, yaw - state->attitude.yaw);

        control.attitude.roll = -vr * MOTOR_THRUST_SCALE;
        control.attitude.pitch = 0;
        control.attitude.yaw = vy * MOTOR_THRUST_SCALE;
        control.thrust = vx * MOTOR_THRUST_SCALE;
    }

    if (supervisorCommandTimeout()) {
        yaw = state->attitude.yaw;
        pidReset(&pid_x);
        pidReset(&pid_y);
        pidReset(&pid_r);
        control.attitude.roll = 0;
        control.attitude.pitch = 0;
        control.attitude.yaw = 0;
        control.thrust = 0;
    }

    *control_out = control;
}