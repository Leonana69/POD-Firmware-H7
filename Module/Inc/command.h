#ifndef __COMMAND_H__
#define __COMMAND_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include "podtp.h"
#include "stabilizer_types.h"

typedef enum {
    COMMAND_STATE_LANDED = 0,
    COMMAND_STATE_NORMAL = 1,
    COMMAND_STATE_LANDING = 2,
} command_state_t;

void commandInit(void);
void commandProcessPacket(PodtpPacket *packet);
void commandSetSetpoint(setpoint_t *s);
void commandGetSetpoint(setpoint_t *s);

command_state_t commandGetState(void);
void commandSetState(command_state_t state);

#ifdef __cplusplus
}
#endif

#endif // __COMMAND_H__