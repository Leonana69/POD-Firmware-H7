#include <string.h>
#include "command.h"
#include "freeRTOS_helper.h"
#include "motor_power.h"
#include "utils.h"
#include "supervisor.h"
#include "debug.h"

STATIC_QUEUE_DEF(commandQueue, 10, setpoint_t);
static setpoint_t currentSetpoint;

static void getHoverSetpoint(setpoint_t *sp, scalar_t height, scalar_t vx, scalar_t vy, scalar_t vyaw) {
    *sp = (setpoint_t) {
        .timestamp = 0,
        .duration = 0,
        .thrust = MOTOR_THRUST_MIN,
        .attitude = { .roll = 0, .pitch = 0, .yaw = 0 },
        .palstance = { .roll = 0, .pitch = 0, .yaw = vyaw },
        .position = { .x = 0, .y = 0, .z = height },
        .velocity = { .x = vx, .y = vy, .z = 0 },
        .mode.x = STABILIZE_VELOCITY,
        .mode.y = STABILIZE_VELOCITY,
        .mode.z = STABILIZE_ABSOLUTE,
        .mode.roll = STABILIZE_DISABLE,
        .mode.pitch = STABILIZE_DISABLE,
        .mode.yaw = STABILIZE_VELOCITY
    };
};

static void getRpytSetpoint(setpoint_t *sp, scalar_t roll, scalar_t pitch, scalar_t yaw, scalar_t thrust) {
    *sp = (setpoint_t) {
        .timestamp = 0,
        .duration = 0,
        .thrust = clamp(thrust, MOTOR_THRUST_MIN, MOTOR_THRUST_MAX),
        .attitude = { .roll = roll, .pitch = pitch, .yaw = yaw },
        .palstance = { .roll = 0, .pitch = 0, .yaw = 0 },
        .position = { .x = 0, .y = 0, .z = 0 },
        .velocity = { .x = 0, .y = 0, .z = 0 },
        .mode.x = STABILIZE_DISABLE,
        .mode.y = STABILIZE_DISABLE,
        .mode.z = STABILIZE_DISABLE,
        .mode.roll = STABILIZE_ABSOLUTE,
        .mode.pitch = STABILIZE_ABSOLUTE,
        .mode.yaw = STABILIZE_ABSOLUTE
    };
};

typedef struct {
    scalar_t height;
    scalar_t vx;
    scalar_t vy;
    scalar_t vyaw;
} __attribute__((packed)) hover_t;

bool commandDecodeHoverPacket(PodtpPacket *packet, setpoint_t *sp) {
    if (packet->length - 1 != sizeof(hover_t)) {
        packet->port = PODTP_PORT_ERROR;
        return false;
    }
    hover_t *hover = (hover_t *)packet->data;
    getHoverSetpoint(sp, hover->height, hover->vx, hover->vy, hover->vyaw);
    return true;
}

typedef struct {
    scalar_t roll;
    scalar_t pitch;
    scalar_t yaw;
    scalar_t thrust;
} __attribute__((packed)) rpyt_t;

bool commandDecodeRpytPacket(PodtpPacket *packet, setpoint_t *sp) {
    if (packet->length - 1 != sizeof(rpyt_t)) {
        packet->port = PODTP_PORT_ERROR;
        return false;
    }
    rpyt_t *rpyt = (rpyt_t *)packet->data;
    getRpytSetpoint(sp, rpyt->roll, rpyt->pitch, rpyt->yaw, rpyt->thrust);
    return true;
}

void commandInit(void) {
    STATIC_QUEUE_INIT(commandQueue);
}

void commandGetSetpoint(setpoint_t *s) {
    if (currentSetpoint.timestamp == 0)
        currentSetpoint.timestamp = osKernelGetTickCount();

    if (currentSetpoint.timestamp + currentSetpoint.duration < osKernelGetTickCount()
        && !STATIC_QUEUE_IS_EMPTY(commandQueue)) {
        STATIC_QUEUE_RECEIVE(commandQueue, &currentSetpoint, 0);
    }
    *s = currentSetpoint;
}

void commandSetSetpoint(setpoint_t *s) {
    STATIC_QUEUE_SEND(commandQueue, s, 0);
}

bool commandProcessPacket(PodtpPacket *packet) {
    static setpoint_t sp;
    bool ret = false;
    memset(&sp, 0, sizeof(setpoint_t));
    switch (packet->port) {
        case PODTP_PORT_RPYT:
            ret = commandDecodeRpytPacket(packet, &sp);
            break;
        case PODTP_PORT_TAKEOFF:
            
            break;
        case PODTP_PORT_LAND:
            
            break;
        case PODTP_PORT_HOVER:
            ret = commandDecodeHoverPacket(packet, &sp);
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