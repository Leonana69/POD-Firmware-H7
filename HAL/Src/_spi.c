#include "_spi.h"

SPI_DMA_READ_WRITE_FUNC_DEF(paa3905, PAA3905_SPI_HANDLE, PAA3905_CS_GPIO_PORT, PAA3905_CS_GPIO_PIN);
SPI_DMA_READ_WRITE_FUNC_DEF(bmi270, BMI270_SPI_HANDLE, BMI270_CS_GPIO_PORT, BMI270_CS_GPIO_PIN);

/*
 * SPI DMA callback functions, give the semaphore
 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == BMI270_SPI_HANDLE.Instance) {
        SPI_DMA_TX_COMPLETE_CALLBACK(bmi270);
    } else if (hspi->Instance == PAA3905_SPI_HANDLE.Instance) {
        SPI_DMA_TX_COMPLETE_CALLBACK(paa3905);
    }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == BMI270_SPI_HANDLE.Instance) {
        SPI_DMA_RX_COMPLETE_CALLBACK(bmi270);
    } else if (hspi->Instance == PAA3905_SPI_HANDLE.Instance) {
        SPI_DMA_RX_COMPLETE_CALLBACK(paa3905);
    }
}

void _SPI_Init() {
    SPI_DMA_READ_WRITE_SEM_INIT(paa3905);
    SPI_DMA_READ_WRITE_SEM_INIT(bmi270);
}