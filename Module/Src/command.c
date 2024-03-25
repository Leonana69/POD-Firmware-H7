#include <string.h>
#include "command.h"
#include "freeRTOS_helper.h"
#include "motor_power.h"
#include "utils.h"
#include "supervisor.h"
#include "debug.h"

STATIC_MUTEX_DEF(setpointMutex);
static setpoint_t setpoint;

void commandInit(void) {
    STATIC_MUTEX_INIT(setpointMutex);
}

void commandGetSetpoint(setpoint_t *s) {
    STATIC_MUTEX_LOCK(setpointMutex, osWaitForever);
    *s = setpoint;
    STATIC_MUTEX_UNLOCK(setpointMutex);
}

void commandSetSetpoint(setpoint_t *s) {
    STATIC_MUTEX_LOCK(setpointMutex, osWaitForever);
    setpoint = *s;
    STATIC_MUTEX_UNLOCK(setpointMutex);
}

typedef struct {
    scalar_t roll;
    scalar_t pitch;
    scalar_t yaw;
    scalar_t thrust;
} __attribute__((packed)) rpyt_t;

bool commandDecodeRpyt(PodtpPacket *packet, setpoint_t *sp) {
    if (packet->length - 1 != sizeof(rpyt_t)) {
        packet->port = PODTP_PORT_ERROR;
        return false;
    }
    rpyt_t *rpyt = (rpyt_t *)packet->data;
    sp->mode.x = sp->mode.y = sp->mode.z = STABILIZE_DISABLE;
    sp->velocity.x = sp->velocity.y = 0;
    sp->mode.roll = sp->mode.pitch = STABILIZE_ABSOLUTE;
    sp->mode.yaw = STABILIZE_VELOCITY;
    sp->attitude.roll = rpyt->roll;
    sp->attitude.pitch = rpyt->pitch;
    sp->palstance.yaw = rpyt->yaw;
    sp->thrust = clamp(rpyt->thrust, MOTOR_THRUST_MIN, MOTOR_THRUST_MAX);
    return true;
}

bool commandProcessPacket(PodtpPacket *packet) {
    static setpoint_t sp;
    bool ret = false;
    memset(&sp, 0, sizeof(setpoint_t));
    switch (packet->port) {
        case PODTP_PORT_RPYT:
            ret = commandDecodeRpyt(packet, &sp);
            break;
        case PODTP_PORT_TAKEOFF:
            
            break;
        case PODTP_PORT_LAND:
            
            break;
        default:
            packet->port = PODTP_PORT_ERROR;
            break;
    }
    if (ret) {
        packet->port = PODTP_PORT_OK;
        supervisorUpdateCommand();
        commandSetSetpoint(&sp);
    }
    packet->type = PODTP_TYPE_ACK;
    packet->length = 1;
    return true;
}