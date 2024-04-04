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
    sp->thrust = 0;
    sp->attitude = (attitude_t) { .roll = 0, .pitch = 0, .yaw = 0 };
    sp->palstance = (palstance_t) { .roll = 0, .pitch = 0, .yaw = vyaw };
    sp->position = (position_t) { .x = 0, .y = 0, .z = height };
    sp->velocity = (velocity_t) { .x = vx, .y = vy, .z = 0 };
    sp->mode.x = STABILIZE_VELOCITY;
    sp->mode.y = STABILIZE_VELOCITY;
    sp->mode.z = STABILIZE_ABSOLUTE;
    sp->mode.roll = STABILIZE_DISABLE;
    sp->mode.pitch = STABILIZE_DISABLE;
    sp->mode.yaw = STABILIZE_VELOCITY;
};

static void getRpytSetpoint(setpoint_t *sp, scalar_t roll, scalar_t pitch, scalar_t yaw, scalar_t thrust) {
    sp->thrust = clamp_f(thrust, MOTOR_THRUST_MIN, MOTOR_THRUST_MAX);
    sp->attitude = (attitude_t) { .roll = roll, .pitch = pitch, .yaw = yaw };
    sp->palstance = (palstance_t) { .roll = 0, .pitch = 0, .yaw = 0 };
    sp->position = (position_t) { .x = 0, .y = 0, .z = 0 };
    sp->velocity = (velocity_t) { .x = 0, .y = 0, .z = 0 };
    sp->mode.x = STABILIZE_DISABLE;
    sp->mode.y = STABILIZE_DISABLE;
    sp->mode.z = STABILIZE_DISABLE;
    sp->mode.roll = STABILIZE_ABSOLUTE;
    sp->mode.pitch = STABILIZE_ABSOLUTE;
    sp->mode.yaw = STABILIZE_ABSOLUTE;
};

static void getXyzySetpoint(setpoint_t *sp, scalar_t x, scalar_t y, scalar_t z, scalar_t yaw) {
    sp->attitude = (attitude_t) { .roll = 0, .pitch = 0, .yaw = yaw };
    sp->palstance = (palstance_t) { .roll = 0, .pitch = 0, .yaw = 0 };
    sp->position = (position_t) { .x = x, .y = y, .z = z };
    sp->velocity = (velocity_t) { .x = 0, .y = 0, .z = 0 };
    sp->mode.x = STABILIZE_ABSOLUTE;
    sp->mode.y = STABILIZE_ABSOLUTE;
    sp->mode.z = STABILIZE_ABSOLUTE;
    sp->mode.roll = STABILIZE_DISABLE;
    sp->mode.pitch = STABILIZE_DISABLE;
    sp->mode.yaw = STABILIZE_ABSOLUTE;
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

typedef struct {
    scalar_t x;
    scalar_t y;
    scalar_t z;
    scalar_t yaw;
} __attribute__((packed)) xyzy_t;

bool commandDecodeXyzyPacket(PodtpPacket *packet, setpoint_t *sp) {
    if (packet->length - 1 != sizeof(rpyt_t)) {
        packet->port = PODTP_PORT_ERROR;
        return false;
    }
    xyzy_t *xyzy = (xyzy_t *)packet->data;
    getXyzySetpoint(sp, xyzy->x, xyzy->y, xyzy->z, xyzy->yaw);
    return true;
}

void commandTakeOff() {
    setpoint_t sp;
    sp.duration = 500;
    getRpytSetpoint(&sp, 0, 0, 0, 20000);
    commandSetSetpoint(&sp);
    getRpytSetpoint(&sp, 0, 0, 0, 40000);
    commandSetSetpoint(&sp);
    getHoverSetpoint(&sp, 1, 0, 0, 0);
    commandSetSetpoint(&sp);
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

    if (supervisorCommandTimeout())
        memset(s, 0, sizeof(setpoint_t));

    *s = currentSetpoint;
}

void commandSetSetpoint(setpoint_t *s) {
    STATIC_QUEUE_SEND(commandQueue, s, 0);
}

void commandProcessPacket(PodtpPacket *packet) {
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
        case PODTP_PORT_POSITION:
            ret = commandDecodeXyzyPacket(packet, &sp);
            break;
        default:
            packet->port = PODTP_PORT_ERROR;
            break;
    }
    if (ret) {
        packet->port = PODTP_PORT_OK;
        supervisorUpdateCommand();
        commandSetSetpoint(&sp);
    } else {
        packet->port = PODTP_PORT_ERROR;
    }
}