#ifndef __MOTOR_2040_H__
#define __MOTOR_2040_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define MOTOR_2040_MIN_SPEED 0.0f
#define MOTOR_2040_MAX_SPEED 2.0f

typedef struct {
    uint16_t speed[4];
} motor_2040_control_t;

void motor2040Init();
void motor2040Send();
void motor2040SetSpeed(uint8_t id, uint16_t speed);
uint16_t motor2040GetSpeed(uint8_t id);

#ifdef __cplusplus
}
#endif

#endif // __MOTOR_2040_H__