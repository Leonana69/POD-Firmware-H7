#ifndef __KALMAN_FILTER_UPDATE_H__
#define __KALMAN_FILTER_UPDATE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stabilizer_types.h"
#include "kalman_core.h"

void kalmanCoreUpdateWithTof(kalmanCoreData_t* coreData, const tof_t *tof, bool isTakingOff);
void kalmanCoreUpdateWithFlow(kalmanCoreData_t* coreData, const flow_t *flow, const vec3f_t *gyro);
void kalmanCoreUpdateWithBaro(kalmanCoreData_t* coreData, const baro_t *baro, bool isFlying);
void kalmanCoreUpdateWithMotor(kalmanCoreData_t* coreData, const motor_t *motor);

#ifdef __cplusplus
}
#endif

#endif // __KALMAN_FILTER_UPDATE_H__