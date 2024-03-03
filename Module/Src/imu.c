#include "imu.h"

#include "bmi270.h"
#include "bmp3.h"
#include "utils.h"
#include "_spi.h"
#include "_i2c.h"

#define MODULE_NAME "IMU"
#include "debug.h"
#include "_config.h"
#include "cmsis_os.h"
#include "freeRTOS_helper.h"
static struct bmi2_dev bmi2Dev;
enum { ACCEL, GYRO };
struct bmi2_sens_config bmi270Config[2];
static float accelValue2Gravity;
static float gyroVale2Degree;

void imuInit() {
    int8_t rslt;
    uint8_t chipId;
    bmi2Dev.delay_us = sensorsDelayUs;
    bmi2Dev.intf = BMI2_SPI_INTF;
    bmi2Dev.read = bmi270_read;
    bmi2Dev.write = bmi270_write;
    rslt = bmi270_init(&bmi2Dev);
    rslt |= bmi2_get_regs(BMI2_CHIP_ID_ADDR, &chipId, 1, &bmi2Dev);
	if (rslt != BMI2_OK || chipId != BMI270_CHIP_ID)
		DEBUG_PRINT("BMI270 Init [FAILED]: rslt: %d\n", rslt);
    else
        DEBUG_PRINT("BMI270 Init [OK]\n");
    bmi2Dev.delay_us(5000, NULL);

    bmi270Config[ACCEL].type = BMI2_ACCEL;
    bmi270Config[GYRO].type = BMI2_GYRO;
    uint8_t sensorsList[2] = { BMI2_ACCEL, BMI2_GYRO };
    rslt = bmi2_sensor_enable(sensorsList, 2, &bmi2Dev);
    /*! Disable power saving mode, this will cause severe delay */
    rslt |= bmi2_set_adv_power_save(BMI2_DISABLE, &bmi2Dev);
    if (rslt != BMI2_OK)
        DEBUG_PRINT("BMI270 Enable [FAILED].\n");
    bmi2Dev.delay_us(5000, NULL);

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
    if (rslt != BMI2_OK)
        DEBUG_PRINT("BMI270 Accel Gyro Meas Config [FAILED].\n");
    bmi2Dev.delay_us(10000, NULL);
}