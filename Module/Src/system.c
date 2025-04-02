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
#include "estimator_kalman.h"
#include "led_seq.h"
#include "link.h"
#include "command.h"
#include "distance.h"
#include "supervisor.h"

STATIC_TASK_DEF(systemTask, SYSTEM_TASK_PRIORITY, SYSTEM_TASK_STACK_SIZE);
STATIC_SEMAPHORE_DEF(systemStart);

void systemInit() {
    _I2C_Init();
    _SPI_Init();
    _TIM_Init();
    _UART_Init();
    STATIC_SEMAPHORE_INIT(systemStart, 1, 0);
    STATIC_TASK_INIT(systemTask, NULL);
}

void systemWaitStart() {
    STATIC_SEMAPHORE_WAIT(systemStart, osWaitForever);
    STATIC_SEMAPHORE_RELEASE(systemStart);
}

void systemTask(void *argument) {
    DEBUG_PRINT("systemTask [START]\n");
    // one-time init tasks
    motorPowerInit();
    commandInit();
    supervisorInit();

    // periodic tasks
    linkInit();
    imuInit();
    baroInit();
    tofInit();
    flowInit();
    stabilizerInit();
    estimatorKalmanInit();
    ledSeqInit();
    distanceInit();
    
    STATIC_SEMAPHORE_RELEASE(systemStart);
    osDelay(osWaitForever);
}