#include "motor_2040.h"
#include "link.h"
#include "stabilizer_types.h"
#include <string.h>

static motor_2040_control_t motorConfig;
void motor2040Init() {
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