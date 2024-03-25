#ifndef __LINK_H__
#define __LINK_H__

#include <stdint.h>
#include <stdbool.h>
#include "podtp.h"
#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

void linkBufferPutChar(uint8_t c);
uint32_t linkInit();
void linkSendPacket(PodtpPacket *packet);
void linkSendData(const uint8_t *data, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif // __LINK_H__