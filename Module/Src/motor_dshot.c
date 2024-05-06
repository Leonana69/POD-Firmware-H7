#include "motor_dshot.h"
#include "tim.h"
#include "_config.h"
#include "config.h"
#include <stdbool.h>

#include "debug.h"

#define DSHOT150_FREQ_KHZ 150
#define DSHOT300_FREQ_KHZ 300
#define DSHOT600_FREQ_KHZ 600

#define DSHOT_DMA_BUFFER_SIZE 20
#define MOTOR_BIT0_PULSE 7
#define MOTOR_BIT1_PULSE 14

typedef struct {
    TIM_HandleTypeDef* tim;
    uint32_t channel;
    int16_t value;
    uint32_t dma_buffer[DSHOT_DMA_BUFFER_SIZE];
} MotorConfig_t;

static MotorConfig_t motorConfig[MOTOR_COUNT] = {
    { &MOTOR_1_TIM, MOTOR_1_CHANNEL, 0, { 0 }},
    { &MOTOR_2_TIM, MOTOR_2_CHANNEL, 0, { 0 }},
    { &MOTOR_3_TIM, MOTOR_3_CHANNEL, 0, { 0 }},
    { &MOTOR_4_TIM, MOTOR_4_CHANNEL, 0, { 0 }}
};

// for reverse ESC installation
// static MotorConfig_t motorConfig[MOTOR_COUNT] = {
//     { &MOTOR_3_TIM, MOTOR_3_CHANNEL, 0, { 0 }},
//     { &MOTOR_4_TIM, MOTOR_4_CHANNEL, 0, { 0 }},
//     { &MOTOR_1_TIM, MOTOR_1_CHANNEL, 0, { 0 }},
//     { &MOTOR_2_TIM, MOTOR_2_CHANNEL, 0, { 0 }},
// };

static void dshot_dma_tc_callback(DMA_HandleTypeDef *hdma) {
	TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
	if (hdma == htim->hdma[TIM_DMA_ID_CC1])
		__HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC1);
	else if(hdma == htim->hdma[TIM_DMA_ID_CC2])
		__HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC2);
	else if(hdma == htim->hdma[TIM_DMA_ID_CC3])
		__HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC3);
	else if(hdma == htim->hdma[TIM_DMA_ID_CC4])
		__HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC4);
}

void motorDShotInit(void) { 
    uint32_t prescaler = (MOTOR_CLOCK_FREQ_KHZ / DSHOT300_FREQ_KHZ / DSHOT_DMA_BUFFER_SIZE) - 1;
    for (int i = 0; i < MOTOR_COUNT; i++) {
        __HAL_TIM_SET_PRESCALER(motorConfig[i].tim, prescaler);
        __HAL_TIM_SET_AUTORELOAD(motorConfig[i].tim, DSHOT_DMA_BUFFER_SIZE - 1);
        motorConfig[i].tim->hdma[motorConfig[i].channel / 4 + 1]->XferCpltCallback = dshot_dma_tc_callback;
        HAL_TIM_PWM_Start(motorConfig[i].tim, motorConfig[i].channel);
    }
}

static void updateDMABuffer(uint32_t* motor_dmabuffer, int16_t value, bool telemetry) {
	uint16_t packet = (value << 1) | (telemetry ? 1 : 0);

	// compute checksum
	uint16_t csum = 0;
	uint16_t csum_data = packet;

	for (int i = 0; i < 3; i++) {
		csum ^=  csum_data; // xor data by nibbles
		csum_data >>= 4;
	}

	csum &= 0xf;
	packet = (packet << 4) | csum;

	for (int i = 0; i < 16; i++) {
		motor_dmabuffer[i] = (packet & 0x8000) ? MOTOR_BIT1_PULSE : MOTOR_BIT0_PULSE;
		packet <<= 1;
	}

    for (int i = 16; i < DSHOT_DMA_BUFFER_SIZE; i++) {
        motor_dmabuffer[i] = 0;
    }
}

void motorDShotWriteDma() {
    for (int i = 0; i < MOTOR_COUNT; i++) {
        updateDMABuffer(motorConfig[i].dma_buffer, motorConfig[i].value, false);
        switch (motorConfig[i].channel) {
            case TIM_CHANNEL_1:
                HAL_DMA_Start_IT(motorConfig[i].tim->hdma[TIM_DMA_ID_CC1], (uint32_t)motorConfig[i].dma_buffer,
                                 (uint32_t)&motorConfig[i].tim->Instance->CCR1, DSHOT_DMA_BUFFER_SIZE);
                __HAL_TIM_ENABLE_DMA(motorConfig[i].tim, TIM_DMA_CC1);
                break;
            case TIM_CHANNEL_2:
                HAL_DMA_Start_IT(motorConfig[i].tim->hdma[TIM_DMA_ID_CC2], (uint32_t)motorConfig[i].dma_buffer,
                                 (uint32_t)&motorConfig[i].tim->Instance->CCR2, DSHOT_DMA_BUFFER_SIZE);
                __HAL_TIM_ENABLE_DMA(motorConfig[i].tim, TIM_DMA_CC2);
                break;
            case TIM_CHANNEL_3:
                HAL_DMA_Start_IT(motorConfig[i].tim->hdma[TIM_DMA_ID_CC3], (uint32_t)motorConfig[i].dma_buffer,
                                 (uint32_t)&motorConfig[i].tim->Instance->CCR3, DSHOT_DMA_BUFFER_SIZE);
                __HAL_TIM_ENABLE_DMA(motorConfig[i].tim, TIM_DMA_CC3);
                break;
            case TIM_CHANNEL_4:
                HAL_DMA_Start_IT(motorConfig[i].tim->hdma[TIM_DMA_ID_CC4], (uint32_t)motorConfig[i].dma_buffer,
                                 (uint32_t)&motorConfig[i].tim->Instance->CCR4, DSHOT_DMA_BUFFER_SIZE);
                __HAL_TIM_ENABLE_DMA(motorConfig[i].tim, TIM_DMA_CC4);
                break;
            default:
                break;
        }
    }
}

void motorDShotSetThrust(uint8_t id, int16_t thrust) {
    motorConfig[id].value = thrust;
}

int16_t motorDShotGetThrust(uint8_t id) {
    return motorConfig[id].value;
}