#include "supervisor.h"

#define Z_ACCEL_THRESHOLD   -0.5f
#define Z_ACCEL_COUNT       30

static bool isTumbled = false;
static bool isLocked = false;

void supervisorUpdate(const imu_t *imu) {
    static uint32_t z_accel_count = 0;
    if (imu->accel.z < Z_ACCEL_THRESHOLD) {
        z_accel_count++;
    } else {
        z_accel_count = 0;
    }

    if (z_accel_count > Z_ACCEL_COUNT) {
        isTumbled = true;
        supervisorLockDrone();
    }
}

bool supervisorCanFly() {
    return !isTumbled && !isLocked;
}

void supervisorUnlockDrone() {
    isLocked = false;
}

void supervisorLockDrone() {
    isLocked = true;
}
