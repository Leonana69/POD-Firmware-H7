#include "tof.h"
#include "_i2c.h"
#include "_config.h"
#include "config.h"
#include "vl53l1.h"
#include "freeRTOS_helper.h"
#include "utils.h"
#include "system.h"

#define MODULE_NAME "TOF"
#include "debug.h"

void tofTask(void *argument);
STATIC_TASK_DEF(tofTask, TOF_TASK_PRIORITY, TOF_TASK_STACK_SIZE);

static VL53L1_Dev_t vl53l1Dev;

void tofInit(void) {
    if (HAL_I2C_IsDeviceReady(&VL53L1_I2C_HANDLE, VL53L1_DEV_ADDR_DEFAULT << 1, 3, 100) != HAL_OK) {
        DEBUG_PRINT("VL53L1 not found\n");
        return;
    }

    vl53l1Dev.read = vl53l1_read_dma;
    vl53l1Dev.write = vl53l1_write_normal;
    vl53l1Dev.delay = sensorsDelayMs;
    vl53l1Dev.millis = sensorsGetMilli;
    vl53l1Dev.i2c_slave_address = VL53L1_DEV_ADDR_DEFAULT;

    if (vl53l1Init(&vl53l1Dev) != VL53L1_ERROR_NONE) {
        DEBUG_PRINT("VL53L1 Init [FAILED]\n");
        return;
    } else {
        DEBUG_PRINT("VL53L1 Init [OK]\n");
        STATIC_TASK_INIT(tofTask, NULL);
    }
}

void tofTask(void *argument) {
    systemWaitStart();
    DEBUG_PRINT("[START]\n");
    tofInit();
    for (;;) {
        osDelay(1000);
    }
}