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
#include "stabilizer_types.h"
#include "estimator_kalman.h"

#define IMU_RATE RATE_1000_HZ

STATIC_MUTEX_DEF(imuDataMutex);
STATIC_TASK_DEF(imuTask, IMU_TASK_PRIORITY, IMU_TASK_STACK_SIZE);

static struct bmi2_dev bmi2Dev;
enum { ACCEL = 0, GYRO = 1 };
struct bmi2_sens_config bmi270Config[2];
static float accelValue2Gravity;
static float gyroVale2Degree;

static imu_t imuData;
static vec3f_t gyroBias;
static float accelScale;

static bool imuCalibration();

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

    STATIC_MUTEX_INIT(imuDataMutex);
    STATIC_TASK_INIT(imuTask, NULL);
    return TASK_INIT_SUCCESS;
}

void imuGet(imu_t *imu) {
    STATIC_MUTEX_LOCK(imuDataMutex, osWaitForever);
    *imu = imuData;
    STATIC_MUTEX_UNLOCK(imuDataMutex);
}

static struct bmi2_sensor_data bmi270Data[2] = {
    { .type = BMI2_ACCEL },
    { .type = BMI2_GYRO }
};

bool imuCalibration() {
    const int caliNbr = 256;
    vec3f_t gyroBiasSquared = (vec3f_t){ 0 };
    gyroBias = (vec3f_t){ 0 };
    accelScale = 0;
    for (int i = 0; i < caliNbr; i++) {
        bmi2_get_sensor_data(&bmi270Data[ACCEL], 1, &bmi2Dev);
		bmi2_get_sensor_data(&bmi270Data[GYRO], 1, &bmi2Dev);
        float x = bmi270Data[ACCEL].sens_data.acc.x;
        float y = bmi270Data[ACCEL].sens_data.acc.y;
        float z = bmi270Data[ACCEL].sens_data.acc.z;
        accelScale += sqrtf(x * x + y * y + z * z);

        gyroBias.x += bmi270Data[GYRO].sens_data.gyr.x;
        gyroBias.y += bmi270Data[GYRO].sens_data.gyr.y;
        gyroBias.z += bmi270Data[GYRO].sens_data.gyr.z;
        gyroBiasSquared.x += bmi270Data[GYRO].sens_data.gyr.x * bmi270Data[GYRO].sens_data.gyr.x;
        gyroBiasSquared.y += bmi270Data[GYRO].sens_data.gyr.y * bmi270Data[GYRO].sens_data.gyr.y;
        gyroBiasSquared.z += bmi270Data[GYRO].sens_data.gyr.z * bmi270Data[GYRO].sens_data.gyr.z;
        osDelay(1);
    }

    accelScale /= caliNbr;
    gyroBias.x /= caliNbr;
    gyroBias.y /= caliNbr;
    gyroBias.z /= caliNbr;
    gyroBiasSquared.x = gyroBiasSquared.x / caliNbr - gyroBias.x * gyroBias.x;
    gyroBiasSquared.y = gyroBiasSquared.y / caliNbr - gyroBias.y * gyroBias.y;
    gyroBiasSquared.z = gyroBiasSquared.z / caliNbr - gyroBias.z * gyroBias.z;
    const float gyroBiasLimit = 1000;
    if (gyroBiasSquared.x < gyroBiasLimit
     && gyroBiasSquared.y < gyroBiasLimit
     && gyroBiasSquared.z < gyroBiasLimit) {
        DEBUG_PRINT("IMU Calibration [OK]: %.3f %.3f %.3f %.3f\n", accelScale, gyroBias.x, gyroBias.y, gyroBias.z);
        return true;
    } else {
        DEBUG_PRINT("IMU Calibration [FAILED]: %.3f %.3f %.3f %.3f\n", accelScale, gyroBias.x, gyroBias.y, gyroBias.z);
        return false;
    }
}

void imuTask(void *argument) {
    estimatorPacket_t packet = { .type = IMU_TASK_INDEX };
    imu_t imuBuffer = { 0 };
    systemWaitStart();
    while (!imuCalibration());
    TASK_TIMER_DEF(IMU, IMU_RATE);
    
    while (1) {
        TASK_TIMER_WAIT(IMU);
        bmi2_get_sensor_data(&bmi270Data[ACCEL], 1, &bmi2Dev);
		bmi2_get_sensor_data(&bmi270Data[GYRO], 1, &bmi2Dev);
        
        imuBuffer.accel.x = bmi270Data[ACCEL].sens_data.acc.x / accelScale;
        imuBuffer.accel.y = bmi270Data[ACCEL].sens_data.acc.y / accelScale;
        imuBuffer.accel.z = bmi270Data[ACCEL].sens_data.acc.z / accelScale;
        imuBuffer.gyro.x = (bmi270Data[GYRO].sens_data.gyr.x - gyroBias.x) * gyroVale2Degree;
        imuBuffer.gyro.y = (bmi270Data[GYRO].sens_data.gyr.y - gyroBias.y) * gyroVale2Degree;
        imuBuffer.gyro.z = (bmi270Data[GYRO].sens_data.gyr.z - gyroBias.z) * gyroVale2Degree;

        STATIC_MUTEX_LOCK(imuDataMutex, osWaitForever);
        imuData = imuBuffer;
        STATIC_MUTEX_UNLOCK(imuDataMutex);
        // TOOD: verify the LPF
        // applyAxis3fLpf((lpf2pData*)(&gyroLpf), &imuBuffer.gyro);

        // TODO: potential alignment calibration

        packet.imu = imuBuffer;
        estimatorKalmanEnqueue(&packet);
    }
}