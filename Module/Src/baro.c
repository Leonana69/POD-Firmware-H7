#define MODULE_NAME "BAR"
#include "debug.h"

#include "baro.h"
#include "config.h"
#include "_i2c.h"
#include "utils.h"
#include "bmp3.h"
#include "estimator_kalman.h"
#include "system.h"

#define BARO_RATE RATE_25_HZ

STATIC_TASK_DEF(baroTask, BARO_TASK_PRIORITY, BARO_TASK_STACK_SIZE);

static uint8_t BMP388_I2C_ADDR = 0x76;
struct bmp3_dev bmp388_dev;

uint32_t baroInit(void) {
    if (HAL_I2C_IsDeviceReady(&BMP388_I2C_HANDLE, BMP388_I2C_ADDR << 1, 3, 100) != HAL_OK) {
        DEBUG_PRINT("BMP388 not found\n");
        return TASK_INIT_FAILED(BARO_TASK_INDEX);
    }

    bmp388_dev.intf = BMP3_I2C_INTF;
    bmp388_dev.read = bmp388_read_dma;
    bmp388_dev.write = bmp388_write_normal;
    bmp388_dev.delay_us = sensorsDelayUs;
    bmp388_dev.intf_ptr = &BMP388_I2C_ADDR;

    int8_t rslt = bmp3_init(&bmp388_dev);
    if (rslt != BMP3_OK) {
        DEBUG_PRINT("BMP388 Init [FAILED]: %d\n", rslt);
        return TASK_INIT_FAILED(BARO_TASK_INDEX);
    }

    struct bmp3_settings settings;
    settings.press_en = BMP3_ENABLE;
    settings.temp_en = BMP3_ENABLE;
    settings.odr_filter.press_os = BMP3_OVERSAMPLING_4X;
    settings.odr_filter.temp_os = BMP3_NO_OVERSAMPLING;
    settings.odr_filter.odr = BMP3_ODR_50_HZ;
    settings.odr_filter.iir_filter = BMP3_IIR_FILTER_COEFF_7;
    settings.op_mode = BMP3_MODE_NORMAL;

    rslt = bmp3_set_sensor_settings(BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN | BMP3_SEL_PRESS_OS 
        | BMP3_SEL_TEMP_OS | BMP3_SEL_ODR | BMP3_SEL_IIR_FILTER, &settings, &bmp388_dev);

    if (rslt != BMP3_OK) {
        DEBUG_PRINT("BMP388 Settings [FAILED]: %d\n", rslt);
        return TASK_INIT_FAILED(BARO_TASK_INDEX);
    }
    
    rslt = bmp3_set_op_mode(&settings, &bmp388_dev);
    if (rslt != BMP3_OK) {
        DEBUG_PRINT("BMP388 Mode [FAILED]: %d\n", rslt);
        return TASK_INIT_FAILED(BARO_TASK_INDEX);
    } else {
        DEBUG_PRINT("BMP388 Init [OK]\n");
        STATIC_TASK_INIT(baroTask, NULL);
    }
    return TASK_INIT_SUCCESS;
}

void baroTask(void *argument) {
    estimatorPacket_t packet = { .type = BARO_TASK_INDEX };
    struct bmp3_data bmp388_data;
    systemWaitStart();
    TASK_TIMER_DEF(BARO, BARO_RATE);
    while (1) {
        TASK_TIMER_WAIT(BARO);
        bmp3_get_sensor_data(BMP3_PRESS | BMP3_TEMP, &bmp388_data, &bmp388_dev);
        packet.baro.pressure = bmp388_data.pressure;
        packet.baro.temperature = bmp388_data.temperature;
        estimatorKalmanEnqueue(&packet);
    }
}