#ifndef ___USART_H__
#define ___USART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "usart.h"
#include "freeRTOS_helper.h"

/*
 * USART DMA dma write function
 */
#define USART_DMA_WRITE_FUNC_DEF(NAME, USART_HANDLE) \
    STATIC_SEMAPHORE_DEF(NAME##_sem); \
    int8_t NAME##_write_dma(const uint8_t *data, uint16_t len) { \
        HAL_StatusTypeDef status; \
        status = HAL_UART_Transmit_DMA(&USART_HANDLE, (uint8_t *) data, len); \
        STATIC_SEMAPHORE_WAIT(NAME##_sem, osWaitForever); \
        return (status == HAL_OK) ? 0 : -1; \
    }
/*
 * UART DMA TX complete callback
 */
#define UART_DMA_TX_COMPLETE_CALLBACK(NAME) \
    STATIC_SEMAPHORE_RELEASE(NAME##_sem);

/*
 * UART DMA TX semaphore initialization
 */
#define UART_DMA_WRITE_SEM_INIT(NAME) \
    STATIC_SEMAPHORE_INIT(NAME##_sem, 1, 0);

#define USART_DMA_WRITE_FUNC_DECL(NAME) \
    int8_t NAME##_write_dma(const uint8_t *data, uint16_t len);

void _UART_Init();
void debugUartTransmit(const uint8_t *data, uint16_t len);
void ESP_UART_HANDLE_IRQHandler();
USART_DMA_WRITE_FUNC_DECL(esp);

#ifdef __cplusplus
}
#endif

#endif // ___USART_H__