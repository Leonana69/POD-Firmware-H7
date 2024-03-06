#include "_i2c.h"
#include <string.h>

I2C_DMA_READ_NORMAL_WRITE_FUNC_DEF(bmp388, BMP388_I2C_HANDLE, uint8_t);
I2C_DMA_READ_NORMAL_WRITE_FUNC_DEF(vl53l1, VL53L1_I2C_HANDLE, uint16_t);

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == VL53L1_I2C_HANDLE.Instance) {
        I2C_DMA_RX_COMPLETE_CALLBACK(vl53l1);
    } else if (hi2c->Instance == BMP388_I2C_HANDLE.Instance) {
        I2C_DMA_RX_COMPLETE_CALLBACK(bmp388);
    }
}

void _I2C_Init() {
    I2C_DMA_READ_WRITE_SEM_INIT(bmp388);
    I2C_DMA_READ_WRITE_SEM_INIT(vl53l1);
}