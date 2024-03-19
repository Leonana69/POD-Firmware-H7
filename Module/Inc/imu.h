#ifndef __IMU_H__
#define __IMU_H__

#ifdef __cplusplus
extern "C" {
#endif

uint32_t imuInit(void);
void imuGet(imu_t *imu);

#ifdef __cplusplus
}
#endif

#endif // __IMU_H__