#ifndef ___I2C_H__
#define ___I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "i2c.h"
#include "_config.h"
#include "freeRTOS_helper.h"

void _I2C_Init();

/*
 * I2C DMA read and normal write function, make sure both DMA and I2C event interrupts are enabled
 */
#define I2C_DMA_READ_NORMAL_WRITE_FUNC_DEF(NAME, I2C_HANDLE, ADDR_TYPE) \
    STATIC_SEMAPHORE_DEF(NAME##_sem); \
    int8_t NAME##_read_dma(ADDR_TYPE reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) { \
        HAL_StatusTypeDef status; \
        uint16_t DevAddress = *(uint8_t*)intf_ptr << 1; \
        uint8_t regAddr[2] = { (reg_addr >> 8) & 0xFF, reg_addr & 0xFF }; \
        uint8_t regAddrOffset = 2 - sizeof(ADDR_TYPE); \
        status = HAL_I2C_Master_Transmit(&I2C_HANDLE, DevAddress, regAddr + regAddrOffset, sizeof(ADDR_TYPE), 100); \
        status |= HAL_I2C_Master_Receive_DMA(&I2C_HANDLE, DevAddress, reg_data, len); \
        STATIC_SEMAPHORE_WAIT(NAME##_sem, osWaitForever); \
        return (status == HAL_OK) ? 0 : -1; \
    } \
    int8_t NAME##_write_normal(ADDR_TYPE reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) { \
        HAL_StatusTypeDef status; \
        uint16_t DevAddress = *(uint8_t*)intf_ptr << 1; \
        static uint8_t sBuffer[256]; \
        memset(sBuffer, 0, 256); \
        sBuffer[0] = reg_addr >> 8; \
        sBuffer[sizeof(ADDR_TYPE) - 1] = reg_addr & 0xff; \
        memcpy(sBuffer + sizeof(ADDR_TYPE), reg_data, len); \
        status = HAL_I2C_Master_Transmit(&I2C_HANDLE, DevAddress, sBuffer, len + sizeof(ADDR_TYPE), 1000); \
        return (status == HAL_OK) ? 0 : -1; \
    }

/*
 * I2C DMA RX complete callback
 */
#define I2C_DMA_RX_COMPLETE_CALLBACK(NAME) \
    STATIC_SEMAPHORE_RELEASE(NAME##_sem);

/*
 * I2C DMA read write function declaration
 */
#define I2C_DMA_READ_NORMAL_WRITE_FUNC_DECL(NAME, ADDR_TYPE) \
    int8_t NAME##_read_dma(ADDR_TYPE reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr); \
    int8_t NAME##_write_normal(ADDR_TYPE reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);

/*
 * I2C DMA RX semaphore initialization
 */
#define I2C_DMA_READ_WRITE_SEM_INIT(NAME) \
    STATIC_SEMAPHORE_INIT(NAME##_sem, 1, 0);

I2C_DMA_READ_NORMAL_WRITE_FUNC_DECL(bmp388, uint8_t);
I2C_DMA_READ_NORMAL_WRITE_FUNC_DECL(vl53l1, uint16_t);

#ifdef __cplusplus
}
#endif

#endif // ___I2C_H__