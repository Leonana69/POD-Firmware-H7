#define MODULE_NAME "IMU"
#include "debug.h"

#include "imu.h"
#include "bmi270.h"
#include "bmp3.h"
#include "utils.h"
#include "_spi.h"
#include "_i2c.h"
#include "freeRTOS_helper.h"
#include "system.h"

STATIC_TASK_DEF(imuTask, IMU_TASK_PRIORITY, IMU_TASK_STACK_SIZE);

static struct bmi2_dev bmi2Dev;
enum { ACCEL, GYRO };
struct bmi2_sens_config bmi270Config[2];
static float accelValue2Gravity;
static float gyroVale2Degree;

uint32_t imuInit() {
    int8_t rslt;
    uint8_t chipId;
    bmi2Dev.delay_us = sensorsDelayUs;
    bmi2Dev.intf = BMI2_SPI_INTF;
    bmi2Dev.read = bmi270_read_dma;
    bmi2Dev.write = bmi270_write_dma;
    bmi2Dev.read_write_len = 32;
    rslt = bmi270_init(&bmi2Dev);
    rslt |= bmi2_get_regs(BMI2_CHIP_ID_ADDR, &chipId, 1, &bmi2Dev);
	if (rslt != BMI2_OK || chipId != BMI270_CHIP_ID) {
        DEBUG_PRINT("BMI270 Init [FAILED]: chipId: 0x%02x, rslt: %d\n", chipId, rslt);
        return TASK_INIT_FAILED(IMU_TASK_INDEX);
    } else
        DEBUG_PRINT("BMI270 Init [OK]\n");

    bmi2Dev.delay_us(10000, NULL);
    bmi270Config[ACCEL].type = BMI2_ACCEL;
    bmi270Config[GYRO].type = BMI2_GYRO;
    uint8_t sensorsList[2] = { BMI2_ACCEL, BMI2_GYRO };
    rslt = bmi2_sensor_enable(sensorsList, 2, &bmi2Dev);
    /*! Disable power saving mode, this will cause severe delay */
    rslt |= bmi2_set_adv_power_save(BMI2_DISABLE, &bmi2Dev);
    if (rslt != BMI2_OK) {
        DEBUG_PRINT("BMI270 Enable [FAILED].\n");
        return TASK_INIT_FAILED(IMU_TASK_INDEX);
    }
    bmi2Dev.delay_us(10000, NULL);

    bmi270Config[ACCEL].cfg.acc.odr = BMI2_ACC_ODR_1600HZ;
    bmi270Config[ACCEL].cfg.acc.range = BMI2_ACC_RANGE_16G;
    bmi270Config[ACCEL].cfg.acc.bwp = BMI2_ACC_OSR4_AVG1;
    bmi270Config[ACCEL].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

    bmi270Config[GYRO].cfg.gyr.odr = BMI2_GYR_ODR_1600HZ;
    bmi270Config[GYRO].cfg.gyr.range = BMI2_GYR_RANGE_2000;
    bmi270Config[GYRO].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;
    bmi270Config[GYRO].cfg.gyr.noise_perf = BMI2_PERF_OPT_MODE;
    bmi270Config[GYRO].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

    accelValue2Gravity = (float)16 / 32768.0f;
    gyroVale2Degree = (float)2000 / 32768.0f;

    rslt = bmi2_set_sensor_config(bmi270Config, 2, &bmi2Dev);
    if (rslt != BMI2_OK) {
        DEBUG_PRINT("BMI270 Accel Gyro Config [FAILED].\n");
        return TASK_INIT_FAILED(IMU_TASK_INDEX);
    }
    bmi2Dev.delay_us(10000, NULL);

    STATIC_TASK_INIT(imuTask, NULL);
    return TASK_INIT_SUCCESS;
}
#include "motor_power.h"
void imuTask(void *argument) {
    systemWaitStart();
    
    struct bmi2_sensor_data bmi270Data[2];
	bmi270Data[ACCEL].type = BMI2_ACCEL;
	bmi270Data[GYRO].type = BMI2_GYRO;
    
    while (1) {
        bmi2_get_sensor_data(&bmi270Data[0], 1, &bmi2Dev);
		bmi2_get_sensor_data(&bmi270Data[1], 1, &bmi2Dev);
        // DEBUG_PRINT("Accel: %d, %d, %d\n", bmi270Data[ACCEL].sens_data.acc.x, bmi270Data[ACCEL].sens_data.acc.y, bmi270Data[ACCEL].sens_data.acc.z);
        // DEBUG_PRINT("Gyro: %d, %d, %d\n", bmi270Data[GYRO].sens_data.gyr.x, bmi270Data[GYRO].sens_data.gyr.y, bmi270Data[GYRO].sens_data.gyr.z);
        osDelay(100);
    }
}