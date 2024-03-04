#ifndef __MOTOR_DSHOT_H__
#define __MOTOR_DSHOT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void motorDShotInit(void);
void motorDShotSetThrust(uint8_t id, uint16_t thrust);
uint16_t motorDShotGetThrust(uint8_t id);

#ifdef __cplusplus
}
#endif

#endif // __MOTOR_DSHOT_H__