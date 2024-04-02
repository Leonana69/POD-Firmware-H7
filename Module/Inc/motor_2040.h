#ifndef __MOTOR_2040_H__
#define __MOTOR_2040_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define MOTOR_2040_MAX_THRUST 2048.0f
#define MOTOR_2040_MIN_THRUST -2048.0f

typedef struct {
    int16_t speed[4];
} motor_2040_control_t;

void motor2040Init();
void motor2040Send();
void motor2040SetSpeed(uint8_t id, int16_t speed);
int16_t motor2040GetSpeed(uint8_t id);

#ifdef __cplusplus
}
#endif

#endif // __MOTOR_2040_H__