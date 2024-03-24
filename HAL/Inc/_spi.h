#ifndef ___SPI_H__
#define ___SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "spi.h"
#include "_config.h"
#include "freeRTOS_helper.h"
#include "debug.h"
#include "led.h"

void _SPI_Init();

/*
 * SPI DMA read write function, make sure both DMA and SPI global interrupts are enabled.
 */
#define SPI_DMA_READ_WRITE_FUNC_DEF(NAME, SPI_HANDLE, CS_GPIO_PORT, CS_GPIO_PIN) \
    STATIC_SEMAPHORE_DEF(NAME##_rx_sem); \
    STATIC_SEMAPHORE_DEF(NAME##_tx_sem); \
    void NAME##_enable_cs() { \
        HAL_GPIO_WritePin(CS_GPIO_PORT, CS_GPIO_PIN, GPIO_PIN_RESET); \
    } \
    void NAME##_disable_cs() { \
        HAL_GPIO_WritePin(CS_GPIO_PORT, CS_GPIO_PIN, GPIO_PIN_SET); \
    } \
    int8_t NAME##_read_dma(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr) { \
        HAL_StatusTypeDef status; \
        NAME##_enable_cs(); \
        while (SPI_HANDLE.State != HAL_SPI_STATE_READY) {} \
        status = HAL_SPI_Transmit_DMA(&SPI_HANDLE, &reg_addr, 1); \
        STATIC_SEMAPHORE_WAIT(NAME##_tx_sem, osWaitForever); \
        while (SPI_HANDLE.State != HAL_SPI_STATE_READY) {} \
        status |= HAL_SPI_Receive_DMA(&SPI_HANDLE, data, len); \
        STATIC_SEMAPHORE_WAIT(NAME##_rx_sem, osWaitForever); \
        NAME##_disable_cs(); \
        return (status == HAL_OK) ? 0 : -1; \
    } \
    int8_t NAME##_write_dma(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr) { \
        HAL_StatusTypeDef status; \
        NAME##_enable_cs(); \
        while (SPI_HANDLE.State != HAL_SPI_STATE_READY) {} \
        status = HAL_SPI_Transmit_DMA(&SPI_HANDLE, &reg_addr, 1); \
        STATIC_SEMAPHORE_WAIT(NAME##_tx_sem, osWaitForever); \
        while (SPI_HANDLE.State != HAL_SPI_STATE_READY) {} \
        status |= HAL_SPI_Transmit_DMA(&SPI_HANDLE, (uint8_t *) data, len); \
        STATIC_SEMAPHORE_WAIT(NAME##_tx_sem, osWaitForever); \
        NAME##_disable_cs(); \
        return (status == HAL_OK) ? 0 : -1; \
    }
    // int8_t NAME##_read_normal(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr) { \
        HAL_StatusTypeDef status; \
        NAME##_enable_cs(); \
        status = HAL_SPI_Transmit(&SPI_HANDLE, &reg_addr, 1, 100); \
        status |= HAL_SPI_Receive(&SPI_HANDLE, data, len, 100); \
        NAME##_disable_cs(); \
        return (status == HAL_OK) ? 0 : -1; \
    } \
    int8_t NAME##_write_normal(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr) { \
        HAL_StatusTypeDef status; \
        NAME##_enable_cs(); \
        status = HAL_SPI_Transmit(&SPI_HANDLE, &reg_addr, 1, 100); \
        status |= HAL_SPI_Transmit(&SPI_HANDLE, (uint8_t *) data, len, 100); \
        NAME##_disable_cs(); \
        return (status == HAL_OK) ? 0 : -1; \
    }

#define SPI_ENABLE_CS(NAME) \
    NAME##_enable_cs()

#define SPI_DISABLE_CS(NAME) \
    NAME##_disable_cs()

/*
 * SPI DMA RX complete callback
 */
#define SPI_DMA_RX_COMPLETE_CALLBACK(NAME) \
    STATIC_SEMAPHORE_RELEASE(NAME##_rx_sem)

/*
 * SPI DMA TX complete callback
 */
#define SPI_DMA_TX_COMPLETE_CALLBACK(NAME) \
    STATIC_SEMAPHORE_RELEASE(NAME##_tx_sem)

/*
 * SPI DMA read write function declaration
 */
#define SPI_DMA_READ_WRITE_FUNC_DECL(NAME) \
    int8_t NAME##_read_dma(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr); \
    int8_t NAME##_write_dma(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr);
    // int8_t NAME##_read_normal(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr); \
    int8_t NAME##_write_normal(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr);
/*
 * SPI DMA TX/RX semaphore initialization
 */
#define SPI_DMA_READ_WRITE_SEM_INIT(NAME) \
    STATIC_SEMAPHORE_INIT(NAME##_rx_sem, 1, 0); \
    STATIC_SEMAPHORE_INIT(NAME##_tx_sem, 1, 0);

SPI_DMA_READ_WRITE_FUNC_DECL(paa3905);
SPI_DMA_READ_WRITE_FUNC_DECL(bmi270);

#ifdef __cplusplus
}
#endif

#endif // ___SPI_H__