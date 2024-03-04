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

// change the clock frequency based on the actual clock frequency
#define MOTOR_COUNT             4
#define MOTOR_CLOCK_FREQ_KHZ    272000
#define MOTOR_1_TIM             htim2
#define MOTOR_2_TIM             htim2
#define MOTOR_3_TIM             htim2
#define MOTOR_4_TIM             htim2
#define MOTOR_1_CHANNEL         TIM_CHANNEL_1
#define MOTOR_2_CHANNEL         TIM_CHANNEL_2
#define MOTOR_3_CHANNEL         TIM_CHANNEL_3
#define MOTOR_4_CHANNEL         TIM_CHANNEL_4

#ifdef __cplusplus
}
#endif

#endif // ___CONFIG_H__