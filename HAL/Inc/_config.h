#ifndef ___CONFIG_H__
#define ___CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif

#define VL53L1_I2C              hi2c3
#define BMP388_I2C              hi2c1
#define BMI270_SPI              hspi1
#define BMI270_CS_GPIO_PORT     GPIOC
#define BMI270_CS_GPIO_PIN      GPIO_PIN_4
#define PAA3905_SPI             hspi2
#define PAA3905_CS_GPIO_PORT    GPIOC
#define PAA3905_CS_GPIO_PIN     GPIO_PIN_6

#ifdef __cplusplus
}
#endif

#endif // ___CONFIG_H__