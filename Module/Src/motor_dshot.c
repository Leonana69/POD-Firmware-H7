#include "motor_dshot.h"
#include "tim.h"
#include "_config.h"
#include <stdbool.h>

#define DSHOT150_FREQ_KHZ 150
#define DSHOT300_FREQ_KHZ 300
#define DSHOT600_FREQ_KHZ 600

#define DSHOT_DMA_BUFFER_SIZE 18
#define MOTOR_BIT0_PULSE 7
#define MOTOR_BIT1_PULSE 14

#define DSHOT_MIN_THRUST 48
#define DSHOT_MAX_THRUST 2047

typedef struct {
    TIM_HandleTypeDef* time;
    uint32_t channel;
    uint16_t value;
    uint32_t dma_buffer[DSHOT_DMA_BUFFER_SIZE];
} MotorConfig_t;

MotorConfig_t motorConfig[MOTOR_COUNT] = {
    { &MOTOR_1_TIM, MOTOR_1_CHANNEL, 0, { 0 }},
    { &MOTOR_2_TIM, MOTOR_2_CHANNEL, 0, { 0 }},
    { &MOTOR_3_TIM, MOTOR_3_CHANNEL, 0, { 0 }},
    { &MOTOR_4_TIM, MOTOR_4_CHANNEL, 0, { 0 }}
};

static void dshot_dma_tc_callback(DMA_HandleTypeDef *hdma) {
    // TODO: test HAL_TIM_OCN_Stop_DMA
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

void motorDShotInit() {
    uint32_t prescaler = (MOTOR_CLOCK_FREQ_KHZ / DSHOT300_FREQ_KHZ / DSHOT_DMA_BUFFER_SIZE) - 1;
    for (int i = 0; i < 4; i++) {
        __HAL_TIM_SET_PRESCALER(motorConfig[i].time, prescaler);
        __HAL_TIM_SET_AUTORELOAD(motorConfig[i].time, DSHOT_DMA_BUFFER_SIZE - 1);
        motorConfig[i].time->hdma[motorConfig[i].channel / 4 + 1]->XferCpltCallback = dshot_dma_tc_callback;
        HAL_TIM_PWM_Start(motorConfig[i].time, motorConfig[i].channel);
    }
}

static void updateDMABuffer(uint32_t* motor_dmabuffer, uint16_t value, bool telemetry) {
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

void motorDShotSetThrust(uint8_t id, uint16_t thrust) {
    // 11 bit value (0~2047) for DShot
    if (thrust > DSHOT_MAX_THRUST)
        thrust = DSHOT_MAX_THRUST;

    if (thrust < DSHOT_MIN_THRUST)
        thrust = DSHOT_MIN_THRUST;

    motorConfig[id].value = thrust;
    updateDMABuffer(motorConfig[id].dma_buffer, thrust, false);
    HAL_TIM_OC_Start_DMA(motorConfig[id].time, motorConfig[id].channel, (uint32_t*)motorConfig[id].dma_buffer, DSHOT_DMA_BUFFER_SIZE);
}

uint16_t motorDShotGetThrust(uint8_t id) {
    return motorConfig[id].value;
}