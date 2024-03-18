#ifndef __PID_H__
#define __PID_H__

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;
    float ki;
    float kd;
    float rate;     // frequency
    float dt;       // 1.0 / rate
    float last_error;
    float integral;

    float i_limit;  // integral limit
    float o_limit;  // output limit
} pid_t;

void pidInit(pid_t *pid, float kp, float ki, float kd, float rate, float i_limit, float o_limit);
float pidUpdate(pid_t *pid, float error);
void pidReset(pid_t *pid);


#ifdef __cplusplus
}
#endif

#endif // __PID_H__