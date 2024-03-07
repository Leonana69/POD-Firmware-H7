#ifndef __KALMAN_FILTER_UPDATE_H__
#define __KALMAN_FILTER_UPDATE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stabilizer_types.h"
#include "kalman_core.h"

void kalmanCoreUpdateWithTof(kalmanCoreData_t* coreData, tofMeasurement_t *tof);
void kalmanCoreUpdateWithFlow(kalmanCoreData_t* coreData, const flowMeasurement_t *flow, const vec3f_t *gyro);
void kalmanCoreUpdateWithBaro(kalmanCoreData_t* coreData, float baroAsl, bool isFlying);

#ifdef __cplusplus
}
#endif

#endif // __KALMAN_FILTER_UPDATE_H__