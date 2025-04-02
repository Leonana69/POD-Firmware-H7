#include "supervisor.h"
#include "cmsis_os.h"
#include "motor_power.h"
#include "freeRTOS_helper.h"
#include "debug.h"
#include "command.h"

#define COMMAND_TIMEOUT     1000
#define Z_ACCEL_THRESHOLD   -0.4f
#define Z_ACCEL_COUNT       100

STATIC_MUTEX_DEF(supervisorMutex);

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

    if (isLocked == false && commandGetState() == COMMAND_STATE_LANDED) {
        DEBUG_REMOTE("** Land [LOCK] **\n");
        supervisorLockDrone(true);
    }
}

void supervisorUpdateCommand() {
    STATIC_MUTEX_LOCK(supervisorMutex, osWaitForever);
    lastCommandTime = osKernelGetTickCount();
    STATIC_MUTEX_UNLOCK(supervisorMutex);
}

bool supervisorCommandTimeout() {
    STATIC_MUTEX_LOCK(supervisorMutex, osWaitForever);
    uint32_t t = lastCommandTime;
    STATIC_MUTEX_UNLOCK(supervisorMutex);
    return osKernelGetTickCount() - t > COMMAND_TIMEOUT;
}

bool supervisorCanFly() {
    return !isTumbled && !isLocked;
}

void supervisorLockDrone(bool lock) {
    if (!lock) {
        isTumbled = false;
        commandSetState(COMMAND_STATE_NORMAL);
    }
    isLocked = lock;
}

void supervisorInit() {
    STATIC_MUTEX_INIT(supervisorMutex);
    isTumbled = false;
    isLocked = true;
    lastCommandTime = osKernelGetTickCount();
}