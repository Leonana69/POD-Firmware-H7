#include "_usart.h"
#include "_config.h"
#include "link.h"
#include "debug.h"

USART_DMA_READ_WRITE_FUNC_DEF(esp, ESP_UART_HANDLE);
#ifdef GEAR
USART_DMA_READ_WRITE_FUNC_DEF(gear, GEAR_UART_HANDLE);
#else
USART_DMA_READ_WRITE_FUNC_DEF(debug, DEBUG_UART_HANDLE);
#endif

#ifdef GEAR
void debugUartTransmit(const uint8_t *data, uint16_t len) {
	HAL_UART_Transmit(&GEAR_UART_HANDLE, data, len, 100);
}
#else
void debugUartTransmit(const uint8_t *data, uint16_t len) {
	debug_write_dma(data, len);
}
#endif


#define ESP_RX_BUFFER_SIZE 128
static uint16_t old_pos = 0;
static uint8_t esp_rx_buffer[ESP_RX_BUFFER_SIZE];

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == ESP_UART_HANDLE.Instance) {
		UART_DMA_TX_COMPLETE_CALLBACK(esp);
	}

#ifdef GEAR
	if (huart->Instance == GEAR_UART_HANDLE.Instance) {
		UART_DMA_TX_COMPLETE_CALLBACK(gear);
	}
#else
	if (huart->Instance == DEBUG_UART_HANDLE.Instance) {
		UART_DMA_TX_COMPLETE_CALLBACK(debug);
	}
#endif
}

void HAL_UART_RxIdleCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == ESP_UART_HANDLE.Instance) {
		uint16_t new_pos = ESP_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(ESP_UART_HANDLE.hdmarx);
		uint16_t data_len;

		if (new_pos >= old_pos) {
			data_len = new_pos - old_pos;
		} else {
			data_len = ESP_RX_BUFFER_SIZE - old_pos + new_pos;
		}

		for (int i = 0; i < data_len; i++) {
			linkBufferPutChar(esp_rx_buffer[(old_pos + i) % ESP_RX_BUFFER_SIZE]);
		}

		old_pos = new_pos;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
#ifdef GEAR
	if (huart->Instance == GEAR_UART_HANDLE.Instance) {
		UART_DMA_RX_COMPLETE_CALLBACK(gear);
	}
#endif
}

void _UART_Init() {
	UART_DMA_READ_WRITE_SEM_INIT(esp);
#ifdef GEAR
	UART_DMA_READ_WRITE_SEM_INIT(gear);
#else
	UART_DMA_READ_WRITE_SEM_INIT(debug);
#endif
	
	HAL_UART_Receive_DMA(&ESP_UART_HANDLE, esp_rx_buffer, ESP_RX_BUFFER_SIZE);
	__HAL_UART_ENABLE_IT(&ESP_UART_HANDLE, UART_IT_IDLE);
}