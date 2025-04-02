#ifndef __SUPERVISOR_H__
#define __SUPERVISOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include "stabilizer_types.h"

void supervisorInit();
void supervisorUpdate(const imu_t *imu);
void supervisorUpdateCommand();
bool supervisorCanFly();
bool supervisorCommandTimeout();
void supervisorLockDrone(bool lock);

#ifdef __cplusplus
}
#endif

#endif //__SUPERVISOR_H__