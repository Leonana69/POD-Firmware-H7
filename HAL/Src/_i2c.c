#include "_i2c.h"
#include "_config.h"

I2C_DMA_READ_NORMAL_WRITE_FUNC_DEF(bmp388, BMP388_I2C, uint8_t);
I2C_DMA_READ_NORMAL_WRITE_FUNC_DEF(vl53l1, VL53L1_I2C, uint16_t);

void _I2C_Init() {
    STATIC_SEMAPHORE_INIT(bmp388_sem, 1, 0);
    STATIC_SEMAPHORE_INIT(vl53l1_sem, 1, 0);
}