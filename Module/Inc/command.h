#ifndef __COMMAND_H__
#define __COMMAND_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include "podtp.h"
#include "stabilizer_types.h"

void commandInit(void);
bool commandProcessPacket(PodtpPacket *packet);
void commandSetSetpoint(setpoint_t *s);
void commandGetSetpoint(setpoint_t *s);

#ifdef __cplusplus
}
#endif

#endif // __COMMAND_H__