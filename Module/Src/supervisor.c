#include "supervisor.h"
#include "cmsis_os.h"
#include "motor_power.h"
#include "debug.h"

#define COMMAND_TIMEOUT     1000
#define Z_ACCEL_THRESHOLD   -0.8f
#define Z_ACCEL_COUNT       100

bool isTumbled = false;
bool isLocked = true;
static uint32_t lastCommandTime = 0;

void supervisorUpdate(const imu_t *imu) {
    static uint32_t z_accel_count = 0;
    if (imu->accel.z < Z_ACCEL_THRESHOLD) {
        z_accel_count++;
    } else {
        z_accel_count = 0;
    }

    if (isTumbled == false && z_accel_count > Z_ACCEL_COUNT) {
        DEBUG_REMOTE("** Tumbled [LOCK] **\n");
        isTumbled = true;
        supervisorLockDrone(true);
    }

    if (isLocked == false && supervisorCommandTimeout()) {
        DEBUG_REMOTE("** Timeout [LOCK] **\n");
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
    return !isTumbled && !isLocked;
}

void supervisorLockDrone(bool lock) {
    if (!lock) {
        isTumbled = false;
        supervisorUpdateCommand();
    }
    isLocked = lock;
}