#ifndef ___I2C_H__
#define ___I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "i2c.h"
#include "freeRTOS_helper.h"

void _I2C_Init();

#define I2C_DMA_READ_NORMAL_WRITE_FUNC_DEF(NAME, I2C_HANDLE, ADDR_TYPE) \
    STATIC_SEMAPHORE_DEF(NAME##_sem); \
    int8_t NAME##_read(ADDR_TYPE reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) { \
        HAL_StatusTypeDef status; \
        uint16_t DevAddress = *(uint8_t*)intf_ptr << 1; \
        status = HAL_I2C_Master_Transmit(&I2C_HANDLE, DevAddress, (uint8_t *) &reg_addr, sizeof(ADDR_TYPE), 100); \
        status |= HAL_I2C_Master_Receive_DMA(&I2C_HANDLE, DevAddress, reg_data, len); \
        STATIC_SEMAPHORE_WAIT(NAME##_sem, osWaitForever); \
        return (status == HAL_OK) ? 0 : -1; \
    } \
    int8_t NAME##_write(ADDR_TYPE reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) { \
        HAL_StatusTypeDef status; \
        uint16_t DevAddress = *(uint8_t*)intf_ptr << 1; \
        status = HAL_I2C_Master_Transmit(&I2C_HANDLE, DevAddress, (uint8_t *) &reg_addr, sizeof(ADDR_TYPE), 100); \
        status |= HAL_I2C_Master_Transmit(&I2C_HANDLE, DevAddress, (uint8_t *) reg_data, len, 100); \
        return (status == HAL_OK) ? 0 : -1; \
    }

#define I2C_DMA_READ_NORMAL_WRITE_FUNC_DECL(NAME, ADDR_TYPE) \
    int8_t NAME##_read(ADDR_TYPE reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr); \
    int8_t NAME##_write(ADDR_TYPE reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);

I2C_DMA_READ_NORMAL_WRITE_FUNC_DECL(bmp388, uint8_t);
I2C_DMA_READ_NORMAL_WRITE_FUNC_DECL(vl53l1, uint16_t);

#ifdef __cplusplus
}
#endif

#endif // ___I2C_H__