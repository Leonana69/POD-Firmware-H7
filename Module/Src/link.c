#define MODULE_NAME "LNK"
#include "debug.h"
#include "assert.h"

#include "link.h"
#include "system.h"
#include "led.h"
#include "command.h"
#include "supervisor.h"
#include "estimator_kalman.h"

STATIC_QUEUE_DEF(linkTxPacketQueue, 15, PodtpPacket);
STATIC_QUEUE_DEF(linkRxPacketQueue, 10, PodtpPacket);
STATIC_TASK_DEF(linkTxTask, LINK_TASK_PRIORITY, LINK_TASK_STACK_SIZE);
STATIC_TASK_DEF(linkRxTask, LINK_TASK_PRIORITY, LINK_TASK_STACK_SIZE);

uint32_t linkInit() {
    STATIC_QUEUE_INIT(linkTxPacketQueue);
    STATIC_QUEUE_INIT(linkRxPacketQueue);
    STATIC_TASK_INIT(linkTxTask, NULL);
    STATIC_TASK_INIT(linkRxTask, NULL);
    DEBUG_PRINT("LINK Init [OK]\n");
    return TASK_INIT_SUCCESS;
}

typedef enum {
    PODTP_STATE_START_1,
    PODTP_STATE_START_2,
    PODTP_STATE_LENGTH,
    PODTP_STATE_RAW_DATA,
    PODTP_STATE_CRC_1,
    PODTP_STATE_CRC_2
} LinkState;

void linkBufferPutChar(uint8_t c) {
    static LinkState state = PODTP_STATE_START_1;
    static uint8_t length = 0;
    static uint8_t check_sum[2] = { 0 };
    static PodtpPacket packet = { 0 };

    switch (state) {
        case PODTP_STATE_START_1:
            if (c == PODTP_START_BYTE_1) {
                state = PODTP_STATE_START_2;
            }
            break;
        case PODTP_STATE_START_2:
            state = (c == PODTP_START_BYTE_2) ? PODTP_STATE_LENGTH : PODTP_STATE_START_1;
            break;
        case PODTP_STATE_LENGTH:
            length = c;
            if (length > PODTP_MAX_DATA_LEN || length == 0) {
                state = PODTP_STATE_START_1;
            } else {
                packet.length = 0;
                check_sum[0] = check_sum[1] = c;
                state = PODTP_STATE_RAW_DATA;
            }
            break;
        case PODTP_STATE_RAW_DATA:
            packet.raw[packet.length++] = c;
            check_sum[0] += c;
            check_sum[1] += check_sum[0];
            if (packet.length >= length) {
                state = PODTP_STATE_CRC_1;
            }
            break;
        case PODTP_STATE_CRC_1:
            state = (c == check_sum[0]) ? PODTP_STATE_CRC_2 : PODTP_STATE_START_1;
            break;
        case PODTP_STATE_CRC_2:
            state = PODTP_STATE_START_1;
            if (c == check_sum[1])
                STATIC_QUEUE_SEND(linkRxPacketQueue, &packet, 0);
            break;
    }
}

void linkSendLog(const uint8_t *data, uint16_t length) {
    linkSendData(PODTP_TYPE_LOG, PORT_LOG_STRING, data, length);
}

void linkSendData(uint8_t type, uint8_t port, const uint8_t *data, uint16_t length) {
    static PodtpPacket packet = { 0 };
    packet.type = type;
    packet.port = port;
    packet.length = length + 1;
    for (uint16_t i = 0; i < length; i++) {
        packet.data[i] = data[i];
    }
    linkSendPacket(&packet);
}

static void linkSetAckPacket(PodtpPacket *packet, uint8_t port) {
    packet->type = PODTP_TYPE_ACK;
    packet->port = port;
    packet->length = 1;
}

bool linkProcessPacket(PodtpPacket *packet) {
    if (packet->length < 1) return false;
    ledToggle(LED_COM);

    // Update the last command time
    supervisorUpdateCommand();

    bool ret = packet->ack;
    switch (packet->type) {
        case PODTP_TYPE_COMMAND:
            commandProcessPacket(packet);
            break;
        case PODTP_TYPE_CTRL:
            if (packet->port == PORT_CTRL_LOCK) {
                if (packet->data[0] == 0) {
                    supervisorLockDrone(false);
                } else if (packet->data[0] == 1) {
                    supervisorLockDrone(true);
                }
                packet->port = PORT_ACK_OK;
            } else if (packet->port == PORT_CTRL_KEEP_ALIVE) {
                // Do nothing
            } else if (packet->port == PORT_CTRL_RESET_ESTIMATOR) {
                DEBUG_PRINT("RESET LOCATION\n");
                estimatorKalmanReset();
            }
            break;
        case PODTP_TYPE_ESP32:
            DEBUG_PRINT("ESP32\n");
            break;
        case PODTP_TYPE_BOOT_LOADER:
            DEBUG_PRINT("BL\n");
            break;
        default:
            break;
    }
    if (ret) {
        linkSetAckPacket(packet, packet->port);
    }
    return ret;
}

void linkSendPacket(PodtpPacket *packet) {
    STATIC_QUEUE_SEND(linkTxPacketQueue, packet, 0);
}

void linkRxTask(void *argument) {
    PodtpPacket packet = { 0 };
    systemWaitStart();
    while (1) {
        STATIC_QUEUE_RECEIVE(linkRxPacketQueue, &packet, osWaitForever);
        if (linkProcessPacket(&packet)) {
            STATIC_QUEUE_SEND(linkTxPacketQueue, &packet, 0);
        }
    }
}

int8_t linkSendPacketUart(PodtpPacket *packet, usart_write_func_t write) {
    static uint8_t buffer[PODTP_MAX_DATA_LEN + 5] = { PODTP_START_BYTE_1, PODTP_START_BYTE_2 };
    uint8_t check_sum[2] = { 0 };
    check_sum[0] = check_sum[1] = packet->length;
    buffer[2] = packet->length;
    for (uint8_t i = 0; i < packet->length; i++) {
        check_sum[0] += packet->raw[i];
        check_sum[1] += check_sum[0];
        buffer[i + 3] = packet->raw[i];
    }
    buffer[packet->length + 3] = check_sum[0];
    buffer[packet->length + 4] = check_sum[1];
    return write(buffer, packet->length + 5);
}

void linkTxTask(void *argument) {
    PodtpPacket packet = { 0 };
    systemWaitStart();

    while (1) {
        STATIC_QUEUE_RECEIVE(linkTxPacketQueue, &packet, osWaitForever);
        linkSendPacketUart(&packet, esp_write_dma);
    }
}