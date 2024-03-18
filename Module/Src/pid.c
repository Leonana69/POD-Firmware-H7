#include "pid.h"
#include "utils.h"

void pidInit(pid_t *pid, float kp, float ki, float kd, float rate, float i_limit, float o_limit) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->rate = rate;
    pid->dt = 1.0f / rate;
    pid->i_limit = i_limit;
    pid->o_limit = o_limit;
    pid->last_error = 0;
    pid->integral = 0;
}

float pidUpdate(pid_t *pid, float error) {
    float output = 0;
    output += pid->kp * error;

    float derivative = (error - pid->last_error) * pid->rate;
    pid->last_error = error;
    output += pid->kd * derivative;

    pid->integral += error * pid->dt;

    if (pid->i_limit > 0) {
        pid->integral = clamp(pid->integral, -pid->i_limit, pid->i_limit);
    }
    output += pid->ki * pid->integral;

    if (pid->o_limit > 0) {
        output = clamp(output, -pid->o_limit, pid->o_limit);
    }
    return output;
}

void pidReset(pid_t *pid) {
    pid->last_error = 0;
    pid->integral = 0;
}