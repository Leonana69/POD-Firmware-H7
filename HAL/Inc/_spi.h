#ifndef ___SPI_H__
#define ___SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "spi.h"

void _SPI_Init();

/*
 * SPI DMA read write function, make sure both DMA and SPI global interrupts are enabled
 */
#define SPI_DMA_READ_WRITE_FUNC_DEF(NAME, SPI_HANDLE, CS_GPIO_PORT, CS_GPIO_PIN) \
    STATIC_SEMAPHORE_DEF(NAME##_rx_sem); \
    STATIC_SEMAPHORE_DEF(NAME##_tx_sem); \
    void NAME##_DMA_RxCpltCallback() { \
        STATIC_SEMAPHORE_RELEASE(NAME##_rx_sem); \
    } \
    void NAME##_DMA_TxCpltCallback() { \
        STATIC_SEMAPHORE_RELEASE(NAME##_tx_sem); \
    } \
    int8_t NAME##_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) { \
        HAL_StatusTypeDef status; \
        HAL_GPIO_WritePin(CS_GPIO_PORT, CS_GPIO_PIN, GPIO_PIN_RESET); \
        reg_addr |= 0x80; \
        while (SPI_HANDLE.State != HAL_SPI_STATE_READY) {} \
        status = HAL_SPI_Transmit_DMA(&SPI_HANDLE, &reg_addr, 1); \
        STATIC_SEMAPHORE_WAIT(NAME##_tx_sem, osWaitForever); \
        while (SPI_HANDLE.State != HAL_SPI_STATE_READY) {} \
        status |= HAL_SPI_Receive_DMA(&SPI_HANDLE, reg_data, len); \
        STATIC_SEMAPHORE_WAIT(NAME##_rx_sem, osWaitForever); \
        HAL_GPIO_WritePin(CS_GPIO_PORT, CS_GPIO_PIN, GPIO_PIN_SET); \
        return (status == HAL_OK) ? 0 : -1; \
    } \
    int8_t NAME##_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) { \
        HAL_StatusTypeDef status; \
        HAL_GPIO_WritePin(CS_GPIO_PORT, CS_GPIO_PIN, GPIO_PIN_RESET); \
        reg_addr &= 0x7F; \
        while (SPI_HANDLE.State != HAL_SPI_STATE_READY) {} \
        status = HAL_SPI_Transmit_DMA(&SPI_HANDLE, &reg_addr, 1); \
        STATIC_SEMAPHORE_WAIT(NAME##_tx_sem, osWaitForever); \
        while (SPI_HANDLE.State != HAL_SPI_STATE_READY) {} \
        status = HAL_SPI_Transmit_DMA(&SPI_HANDLE, (uint8_t *) reg_data, len); \
        STATIC_SEMAPHORE_WAIT(NAME##_tx_sem, osWaitForever); \
        HAL_GPIO_WritePin(CS_GPIO_PORT, CS_GPIO_PIN, GPIO_PIN_SET); \
        return (status == HAL_OK) ? 0 : -1; \
    }

#define SPI_DMA_RX_COMPLETE_CALLBACK(NAME) \
    NAME##_DMA_RxCpltCallback();

#define SPI_DMA_TX_COMPLETE_CALLBACK(NAME) \
    NAME##_DMA_TxCpltCallback();

#define SPI_DMA_READ_WRITE_FUNC_DECL(NAME) \
    void NAME##_DMA_RxCpltCallback(); \
    void NAME##_DMA_TxCpltCallback(); \
    int8_t NAME##_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr); \
    int8_t NAME##_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);

#define SPI_DMA_READ_WRITE_SEM_INIT(NAME) \
    STATIC_SEMAPHORE_INIT(NAME##_rx_sem, 1, 0); \
    STATIC_SEMAPHORE_INIT(NAME##_tx_sem, 1, 0);

SPI_DMA_READ_WRITE_FUNC_DECL(paa3905);
SPI_DMA_READ_WRITE_FUNC_DECL(bmi270);

#ifdef __cplusplus
}
#endif

#endif // ___SPI_H__