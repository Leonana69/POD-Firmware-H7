#include "supervisor.h"
#include "cmsis_os.h"
#include "motor_power.h"

#define COMMAND_TIMEOUT     500
#define Z_ACCEL_THRESHOLD   -0.5f
#define Z_ACCEL_COUNT       30

static bool isTumbled = false;
static bool isLocked = true;
static uint32_t lastCommandTime = 0;

void supervisorUpdate(const imu_t *imu) {
    static uint32_t z_accel_count = 0;
    if (imu->accel.z < Z_ACCEL_THRESHOLD) {
        z_accel_count++;
    } else {
        z_accel_count = 0;
    }

    if (z_accel_count > Z_ACCEL_COUNT) {
        isTumbled = true;
        supervisorLockDrone(true);
    }
}

void supervisorUpdateCommand() {
    lastCommandTime = osKernelGetTickCount();
}

bool supervisorCommandTimeout() {
    return osKernelGetTickCount() - lastCommandTime > COMMAND_TIMEOUT;
}

bool supervisorCanFly() {
    return !isTumbled
        && !isLocked
        && !(motorPowerIsFlying() && supervisorCommandTimeout());
}

void supervisorLockDrone(bool lock) {
    isLocked = lock;
}
