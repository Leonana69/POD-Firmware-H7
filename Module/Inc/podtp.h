#ifndef __PODTP_H__
#define __PODTP_H__

#include <stdint.h>

#define PODTP_MAX_DATA_LEN 127

#define PODTP_START_BYTE_1 0xAD
#define PODTP_START_BYTE_2 0x6E

enum {
    PODTP_TYPE_ERROR = 0x0,
    PODTP_TYPE_ACK = 0x1,
    PODTP_TYPE_COMMAND = 0x2,
    PODTP_TYPE_ESP32 = 0xE,
    PODTP_TYPE_BOOT_LOADER = 0xF,
};

typedef struct {
    uint8_t length;
    union {
        struct {
            union {
                uint8_t header;
                struct {
                    uint8_t port:4;
                    uint8_t type:4;
                };
            };
            uint8_t data[PODTP_MAX_DATA_LEN];
        } __attribute__((packed));
        uint8_t raw[PODTP_MAX_DATA_LEN + 1];
    };
} PodtpPacket;

#endif // __PODTP_H__