#ifndef __LINK_H__
#define __LINK_H__

#include <stdint.h>
#include <stdbool.h>
#include "podtp.h"
#include "config.h"
#include "_usart.h"

#ifdef __cplusplus
extern "C" {
#endif

void linkBufferPutChar(uint8_t c);
uint32_t linkInit();
void linkSendPacket(PodtpPacket *packet);
void linkSendLog(const uint8_t *data, uint16_t length);
void linkSendData(uint8_t type, uint8_t port, const uint8_t *data, uint16_t length);
int8_t linkSendPacketUart(PodtpPacket *packet, usart_write_func_t write);

#ifdef __cplusplus
}
#endif

#endif // __LINK_H__