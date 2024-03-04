#include "_spi.h"
#include "_config.h"

SPI_DMA_READ_WRITE_FUNC_DEF(paa3905, PAA3905_SPI, PAA3905_CS_GPIO_PORT, PAA3905_CS_GPIO_PIN);
SPI_DMA_READ_WRITE_FUNC_DEF(bmi270, BMI270_SPI, BMI270_CS_GPIO_PORT, BMI270_CS_GPIO_PIN);

void _SPI_Init() {
    SPI_DMA_READ_WRITE_SEM_INIT(paa3905);
    SPI_DMA_READ_WRITE_SEM_INIT(bmi270);
}