#include "pid.h"
#include "utils.h"
#include "arm_math.h"

void pidInit(pid_t *pid, float kp, float ki, float kd, float rate, float cutoff_freq, float i_limit, float o_limit) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->rate = rate;
    pid->dt = 1.0f / rate;
    pid->i_limit = i_limit;
    pid->o_limit = o_limit;
    pid->last_error = 0;
    pid->integral = 0;
    pid->filtered_derivative = 0;
    pid->tau = 1 / (2 * PI * cutoff_freq);
}

float pidUpdate(pid_t *pid, float error) {
    float output = 0;
    output += pid->kp * error;

    float raw_derivative = (error - pid->last_error) * pid->rate;
    pid->last_error = error;

    pid->filtered_derivative = pid->filtered_derivative * pid->tau + raw_derivative * (1 - pid->tau);
    
    output += pid->kd * pid->filtered_derivative;

    pid->integral += error * pid->dt;

    if (pid->i_limit > 0) {
        pid->integral = clamp_f(pid->integral, -pid->i_limit, pid->i_limit);
    }
    output += pid->ki * pid->integral;

    if (pid->o_limit > 0) {
        output = clamp_f(output, -pid->o_limit, pid->o_limit);
    }
    return output;
}

void pidReset(pid_t *pid) {
    pid->last_error = 0;
    pid->integral = 0;
}