#ifndef __ESTIMATOR_ATTITUDE_H__
#define __ESTIMATOR_ATTITUDE_H__

#include "stabilizer_types.h"
#include "kalman_core.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t type;
    union {
        tof_t tof;
        flow_t flow;
        imu_t imu;
        baro_t baro;
        motor_t motor;
    };
} estimatorPacket_t;

uint32_t estimatorKalmanInit();
void estimatorKalmanUpdate(state_t *state);
void estimatorKalmanEnqueue(estimatorPacket_t *packet);
void estimatorKalmanSetBaroReference(float height);
void estimatorKalmanGetCoreData(kalmanCoreData_t *data);
void estimatorKalmanReset();

#ifdef __cplusplus
}
#endif

#endif // __ESTIMATOR_ATTITUDE_H__