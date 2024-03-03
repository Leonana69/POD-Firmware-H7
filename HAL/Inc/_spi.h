#ifndef ___SPI_H__
#define ___SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "spi.h"

void _SPI_Init();

#define SPI_DMA_READ_NORMAL_WRITE_FUNC_DEF(NAME, SPI_HANDLE, CS_GPIO_PORT, CS_GPIO_PIN) \
    int8_t NAME##_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) { \
        HAL_StatusTypeDef status; \
        HAL_GPIO_WritePin(CS_GPIO_PORT, CS_GPIO_PIN, GPIO_PIN_RESET); \
        status = HAL_SPI_TransmitReceive_DMA(&SPI_HANDLE, writeBuffer, readBuffer, size); \
        HAL_GPIO_WritePin(CS_GPIO_PORT, CS_GPIO_PIN, GPIO_PIN_SET); \
        return (status == HAL_OK) ? 0 : -1; \
    } \
    int8_t NAME##_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) { \
        HAL_StatusTypeDef status; \
        HAL_GPIO_WritePin(CS_GPIO_PORT, CS_GPIO_PIN, GPIO_PIN_RESET); \
        status = HAL_SPI_Transmit(&SPI_HANDLE, reg_data, len, 100); \
        HAL_GPIO_WritePin(CS_GPIO_PORT, CS_GPIO_PIN, GPIO_PIN_SET); \
        return (status == HAL_OK) ? 0 : -1; \
    }

#define SPI_DMA_READ_NORMAL_WRITE_FUNC_DECL(NAME) \
    int8_t NAME##_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr); \
    int8_t NAME##_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);

SPI_DMA_READ_NORMAL_WRITE_FUNC_DECL(paa3905);

#ifdef __cplusplus
}
#endif

#endif // ___SPI_H__