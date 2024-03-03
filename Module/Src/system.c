#include "system.h"
#include "_i2c.h"
#include "_spi.h"
#include "config.h"
#include "freeRTOS_helper.h"
#include "imu.h"

#define MODULE_NAME "SYSTEM"
#include "debug.h"

void systemTask(void *argument);
STATIC_TASK_DEF(systemTask, SYSTEM_TASK_PRIORITY, SYSTEM_TASK_STACK_SIZE);

void systemInit() {
    _I2C_Init();
    _SPI_Init();
    STATIC_TASK_INIT(systemTask, NULL);
}

void systemTask(void *argument) {
    DEBUG_PRINT("systemTask [START]\n");
    imuInit();
    for (;;) {
        osDelay(1000);
    }
}