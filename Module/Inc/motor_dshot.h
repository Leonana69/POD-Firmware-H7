#ifndef __MOTOR_DSHOT_H__
#define __MOTOR_DSHOT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// TODO: test this base value
#define MOTOR_DSHOT_MAX_THRUST  2047
#define MOTOR_DSHOT_MIN_THRUST  48
#define MOTOR_DSHOT_BASE_THRUST 10000.0f
#define MOTOR_DSHOT_MIN_VALUE   1000.0f
#define MOTOR_DSHOT_MAX_VALUE  25000.0f

void motorDShotInit(void);
void motorDShotSetThrust(uint8_t id, int16_t thrust);
void motorDShotWriteDma();
int16_t motorDShotGetThrust(uint8_t id);

#ifdef __cplusplus
}
#endif

#endif // __MOTOR_DSHOT_H__