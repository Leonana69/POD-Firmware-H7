#include "motor_2040.h"
#include "link.h"
#include "stabilizer_types.h"
#include <string.h>
#include "_usart.h"
#include "debug.h"
#include "system.h"
#include "utils.h"
#include "estimator_kalman.h"

/* These two params need calibration */
#define DX_COEFF 34000.0f
#define DY_COEFF 36000.0f

STATIC_TASK_DEF(motor2040Task, MOTOR_2040_TASK_PRIORITY, MOTOR_2040_TASK_STACK_SIZE);

static motor_2040_control_t motorConfig;
#define MOTOR_2040_PACKET_LENGTH (sizeof(motor_encoder_t) + 4)
void motor2040Init() {
    STATIC_TASK_INIT(motor2040Task, NULL);
}

void motor2040Send() {
    static PodtpPacket packet = {
        .header = 0x40,
        .data = {0}
    };
    packet.length = sizeof(motorConfig) + 1;
    memcpy(packet.data, &motorConfig, sizeof(motorConfig));
#ifdef GEAR
    linkSendPacketUart(&packet, gear_write_dma);
#endif
}

void motor2040SetSpeed(uint8_t id, int16_t speed) {
    motorConfig.speed[id] = speed;
}

int16_t motor2040GetSpeed(uint8_t id) {
    return motorConfig.speed[id];
}

typedef enum {
    MOTOR_2040_STATE_START_1,
    MOTOR_2040_STATE_START_2,
    MOTOR_2040_STATE_RAW_DATA,
    MOTOR_2040_STATE_CRC_1,
    MOTOR_2040_STATE_CRC_2
} LinkState;

static motor_encoder_t motorEncoder;
bool motor2040ProcessEncoder(uint8_t *buffer) {
    static LinkState state = MOTOR_2040_STATE_START_1;
    static uint8_t *data;
    static uint8_t check_sum[2] = { 0 };
    static uint8_t index = 0;

    bool result = false;

    for (int i = 0; i < MOTOR_2040_PACKET_LENGTH; i++) {
        switch (state) {
            case MOTOR_2040_STATE_START_1:
                if (buffer[i] == PODTP_START_BYTE_1) {
                    state = MOTOR_2040_STATE_START_2;
                }
                break;
            case MOTOR_2040_STATE_START_2:
                state = (buffer[i] == PODTP_START_BYTE_2) ? MOTOR_2040_STATE_RAW_DATA : MOTOR_2040_STATE_START_1;
                data = (uint8_t *)&motorEncoder;
                index = 0;
                check_sum[0] = check_sum[1] = 0;
                break;
            case MOTOR_2040_STATE_RAW_DATA:
                data[index++] = buffer[i];
                check_sum[0] += buffer[i];
                check_sum[1] += check_sum[0];
                if (index == sizeof(motor_encoder_t)) {
                    state = MOTOR_2040_STATE_CRC_1;
                }
                break;
            case MOTOR_2040_STATE_CRC_1:
                state = (buffer[i] == check_sum[0]) ? MOTOR_2040_STATE_CRC_2 : MOTOR_2040_STATE_START_1;
                break;
            case MOTOR_2040_STATE_CRC_2:
                state = MOTOR_2040_STATE_START_1;
                if (buffer[i] == check_sum[1])
                    result = true;
                break;
            default:
                break;
        }
    }
    return result;
}

void motor2040Task(void *argument) {
    estimatorPacket_t packet = { .type = MOTOR_2040_TASK_INDEX };
    uint8_t buffer[32];
    systemWaitStart();
    bool isInit = false;
    motor_encoder_t prevEncoder = { 0 };
    motor_encoder_t diff = { 0 };
    uint32_t lastTime = getTimeUs();
    // int count = 0;
    while (1) {
        gear_read_dma(buffer, MOTOR_2040_PACKET_LENGTH);
        if (motor2040ProcessEncoder(buffer)) {
            if (!isInit) {
                memcpy(&prevEncoder, &motorEncoder, sizeof(motor_encoder_t));
                isInit = true;
            }
            uint32_t currentTime = getTimeUs();
            uint32_t deltaTime = getDurationUs(lastTime, currentTime);
            lastTime = currentTime;
            for (int i = 0; i < 4; i++) {
                diff.count[i] = (motorEncoder.count[i] - prevEncoder.count[i]);
            }
            memcpy(&prevEncoder, &motorEncoder, sizeof(motor_encoder_t));
            float dx = (diff.count[0] + diff.count[1] + diff.count[2] + diff.count[3]) / DX_COEFF;
            float dy = (diff.count[0] - diff.count[1] + diff.count[2] - diff.count[3]) / DY_COEFF;
            packet.motor.dx = dx;
            packet.motor.dy = dy;
            packet.motor.stdDevX = 0.005;
            packet.motor.stdDevY = 0.005;
            packet.motor.dt = deltaTime / 1e6f;
            estimatorKalmanEnqueue(&packet);
        }
        osDelay(5);
    }
}