#ifndef __MOTOR_DSHOT_H__
#define __MOTOR_DSHOT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// TODO: test this base value
#define DSHOT_MAX_THRUST 2047
#define DSHOT_MIN_THRUST 48

void motorDShotInit(void);
void motorDShotSetThrust(uint8_t id, int16_t thrust);
void motorDShotWriteDma();
int16_t motorDShotGetThrust(uint8_t id);

#ifdef __cplusplus
}
#endif

#endif // __MOTOR_DSHOT_H__