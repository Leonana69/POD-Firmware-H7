#ifndef __IMU_H__
#define __IMU_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stabilizer_types.h"

uint32_t imuInit(void);
void imuGetData(imu_t *imu);
void imuWaitData();

#ifdef __cplusplus
}
#endif

#endif // __IMU_H__