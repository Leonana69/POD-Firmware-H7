#define MODULE_NAME "SYS"
#include "debug.h"

#include "system.h"
#include "_i2c.h"
#include "_spi.h"
#include "_usart.h"
#include "_tim.h"
#include "config.h"
#include "freeRTOS_helper.h"
#include "imu.h"
#include "baro.h"
#include "motor_power.h"
#include "led.h"
#include "tof.h"
#include "flow.h"
#include "stabilizer.h"
#include "controller_pid.h"
#include "estimator_kalman.h"
#include "led_seq.h"
#include "link.h"

STATIC_TASK_DEF(systemTask, SYSTEM_TASK_PRIORITY, SYSTEM_TASK_STACK_SIZE);
STATIC_SEMAPHORE_DEF(systemStart);

static bool isInit = false;

void systemInit() {
    _I2C_Init();
    _SPI_Init();
    _TIM_Init();
    _UART_Init();
    STATIC_SEMAPHORE_INIT(systemStart, 1, 0);
    STATIC_TASK_INIT(systemTask, NULL);
}

void systemWaitStart() {
    while (!isInit) { osDelay(10); }
    STATIC_SEMAPHORE_WAIT(systemStart, osWaitForever);
    STATIC_SEMAPHORE_RELEASE(systemStart);
}

void systemTask(void *argument) {
    DEBUG_PRINT("systemTask [START]\n");
    // one-time init tasks
    motorPowerInit();

    // periodic tasks
    imuInit();
    baroInit();
    tofInit();
    flowInit();
    stabilizerInit();
    estimatorKalmanInit();
    ledSeqInit();
    linkInit();
    
    STATIC_SEMAPHORE_RELEASE(systemStart);
    isInit = true;
    for (;;) { osDelay(osWaitForever); }
}