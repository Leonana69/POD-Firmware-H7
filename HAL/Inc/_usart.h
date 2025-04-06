#ifndef ___USART_H__
#define ___USART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "usart.h"
#include "freeRTOS_helper.h"

typedef int8_t (*usart_write_func_t)(const uint8_t *data, uint16_t len);

/*
 * USART DMA dma write function
 */
#define USART_DMA_READ_WRITE_FUNC_DEF(NAME, USART_HANDLE) \
    STATIC_SEMAPHORE_DEF(NAME##_tx_sem); \
    STATIC_SEMAPHORE_DEF(NAME##_rx_sem); \
    int8_t NAME##_write_dma(const uint8_t *data, uint16_t len) { \
        HAL_StatusTypeDef status; \
        status = HAL_UART_Transmit_DMA(&USART_HANDLE, (uint8_t *) data, len); \
        STATIC_SEMAPHORE_WAIT(NAME##_tx_sem, osWaitForever); \
        return (status == HAL_OK) ? 0 : -1; \
    } \
    int8_t NAME##_read_dma(uint8_t *data, uint16_t len) { \
        HAL_StatusTypeDef status; \
        status = HAL_UART_Receive_DMA(&USART_HANDLE, data, len); \
        STATIC_SEMAPHORE_WAIT(NAME##_rx_sem, osWaitForever); \
        return (status == HAL_OK) ? 0 : -1; \
    }
/*
 * UART DMA TX complete callback
 */
#define UART_DMA_TX_COMPLETE_CALLBACK(NAME) \
    STATIC_SEMAPHORE_RELEASE(NAME##_tx_sem);

/*
 * UART DMA RX complete callback
 */
#define UART_DMA_RX_COMPLETE_CALLBACK(NAME) \
    STATIC_SEMAPHORE_RELEASE(NAME##_rx_sem);

/*
 * UART DMA TX semaphore initialization
 */
#define UART_DMA_READ_WRITE_SEM_INIT(NAME) \
    STATIC_SEMAPHORE_INIT(NAME##_tx_sem, 1, 0); \
    STATIC_SEMAPHORE_INIT(NAME##_rx_sem, 1, 0);

#define USART_DMA_WRITE_FUNC_DECL(NAME) \
    int8_t NAME##_write_dma(const uint8_t *data, uint16_t len); \
    int8_t NAME##_read_dma(uint8_t *data, uint16_t len);

void _UART_Init();
void debugUartTransmit(const uint8_t *data, uint16_t len);
USART_DMA_WRITE_FUNC_DECL(esp);
USART_DMA_WRITE_FUNC_DECL(debug);

#ifdef GEAR
USART_DMA_WRITE_FUNC_DECL(gear);
#endif

void HAL_UART_RxIdleCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif // ___USART_H__