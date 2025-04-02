#include <string.h>
#include "command.h"
#include "freeRTOS_helper.h"
#include "motor_power.h"
#include "utils.h"
#include "supervisor.h"
#include "debug.h"
#include "stabilizer_types.h"
#include "estimator_kalman.h"

STATIC_QUEUE_DEF(commandQueue, 20, setpoint_t);
static setpoint_t currentSetpoint;

static command_state_t commandState = COMMAND_STATE_LANDED;

static void getHoverSetpoint(setpoint_t *sp, scalar_t vx, scalar_t vy, scalar_t vyaw, scalar_t height) {
    sp->thrust = 0;
    sp->palstance = (palstance_t) { .roll = 0, .pitch = 0, .yaw = vyaw };
    sp->velocity = (velocity_t) { .x = vx, .y = vy, .z = 0 };
    sp->position.z = height;
    sp->velocity_body = true;
    sp->mode.x = STABILIZE_VELOCITY;
    sp->mode.y = STABILIZE_VELOCITY;
    sp->mode.z = STABILIZE_ABSOLUTE;
    sp->mode.roll = STABILIZE_DISABLE;
    sp->mode.pitch = STABILIZE_DISABLE;
    sp->mode.yaw = STABILIZE_VELOCITY;
};

static void getVelocitySetpoint(setpoint_t *sp, scalar_t vx, scalar_t vy, scalar_t vyaw, scalar_t vz) {
    sp->thrust = 0;
    sp->palstance = (palstance_t) { .roll = 0, .pitch = 0, .yaw = vyaw };
    sp->velocity = (velocity_t) { .x = vx, .y = vy, .z = vz };
    sp->velocity_body = true;
    sp->mode.x = STABILIZE_VELOCITY;
    sp->mode.y = STABILIZE_VELOCITY;
    sp->mode.z = STABILIZE_VELOCITY;
    sp->mode.roll = STABILIZE_DISABLE;
    sp->mode.pitch = STABILIZE_DISABLE;
    sp->mode.yaw = STABILIZE_VELOCITY;
};

static void getRpytSetpoint(setpoint_t *sp, scalar_t roll, scalar_t pitch, scalar_t yaw, scalar_t thrust) {
    sp->thrust = clamp_f(thrust, 0, motorPowerGetMaxThrust());
    sp->attitude = (attitude_t) { .roll = roll, .pitch = pitch, .yaw = yaw };
    sp->velocity = (velocity_t) { .x = 0, .y = 0, .z = 0 };
    sp->velocity_body = false;
    sp->mode.x = STABILIZE_DISABLE;
    sp->mode.y = STABILIZE_DISABLE;
    sp->mode.z = STABILIZE_DISABLE;
    sp->mode.roll = STABILIZE_ABSOLUTE;
    sp->mode.pitch = STABILIZE_ABSOLUTE;
    sp->mode.yaw = STABILIZE_ABSOLUTE;
};

static void getXyzySetpoint(setpoint_t *sp, scalar_t x, scalar_t y, scalar_t z, scalar_t yaw) {
    state_t state;
    estimatorKalmanUpdate(&state);
    sp->thrust = 0;
    sp->attitude = (attitude_t) { .roll = 0, .pitch = 0, .yaw = yaw + state.attitude.yaw };
    sp->position = (position_t) { .x = x, .y = y, .z = z };
    sp->velocity_body = false;
    sp->mode.x = STABILIZE_ABSOLUTE;
    sp->mode.y = STABILIZE_ABSOLUTE;
    sp->mode.z = STABILIZE_ABSOLUTE;
    sp->mode.roll = STABILIZE_DISABLE;
    sp->mode.pitch = STABILIZE_DISABLE;
    sp->mode.yaw = STABILIZE_ABSOLUTE;
};

typedef struct {
    scalar_t vx;
    scalar_t vy;
    scalar_t vyaw;
    scalar_t height;
} __attribute__((packed)) hover_t;
static void commandDecodeHoverPacket(PodtpPacket *packet, setpoint_t *sp) {
    if (packet->length - 1 != sizeof(hover_t)) {
        packet->port = PORT_ACK_ERROR;
        return;
    }
    hover_t *hover = (hover_t *)packet->data;
    getHoverSetpoint(sp, hover->vx, hover->vy, hover->vyaw, hover->height);
    commandSetSetpoint(sp);
}

typedef struct {
    scalar_t vx;
    scalar_t vy;
    scalar_t vyaw;
    scalar_t vz;
} __attribute__((packed)) vxyzy_t;
static void commandDecodeVelocityPacket(PodtpPacket *packet, setpoint_t *sp) {
    if (packet->length - 1 != sizeof(vxyzy_t)) {
        packet->port = PORT_ACK_ERROR;
        return;
    }
    vxyzy_t *vxyzy = (vxyzy_t *)packet->data;
    getVelocitySetpoint(sp, vxyzy->vx, vxyzy->vy, vxyzy->vyaw, vxyzy->vz);
    commandSetSetpoint(sp);
}

typedef struct {
    scalar_t roll;
    scalar_t pitch;
    scalar_t yaw;
    scalar_t thrust;
} __attribute__((packed)) rpyt_t;
static void commandDecodeRpytPacket(PodtpPacket *packet, setpoint_t *sp) {
    if (packet->length - 1 != sizeof(rpyt_t)) {
        packet->port = PORT_ACK_ERROR;
        return;
    }
    rpyt_t *rpyt = (rpyt_t *)packet->data;
    getRpytSetpoint(sp, rpyt->roll, rpyt->pitch, rpyt->yaw, rpyt->thrust);
    commandSetSetpoint(sp);
}

typedef struct {
    scalar_t x;
    scalar_t y;
    scalar_t z;
    scalar_t yaw;
} __attribute__((packed)) xyzy_t;
static void commandDecodeXyzyPacket(PodtpPacket *packet, setpoint_t *sp) {
    if (packet->length - 1 != sizeof(rpyt_t)) {
        packet->port = PORT_ACK_ERROR;
        return;
    }
    xyzy_t *xyzy = (xyzy_t *)packet->data;
    getXyzySetpoint(sp, xyzy->x, xyzy->y, xyzy->z, xyzy->yaw);
    commandSetSetpoint(sp);
}

static void commandTakeOff() {
    setpoint_t _sp;
    _sp.timestamp = 0;
    _sp.duration = 500;
    getRpytSetpoint(&_sp, 0, 0, 0, 11600);
    commandSetSetpoint(&_sp);

    _sp.duration = 1000;
    getRpytSetpoint(&_sp, 0, 0, 0, 12000);
    commandSetSetpoint(&_sp);

    // keep hovering
    _sp.duration = 0;
    getHoverSetpoint(&_sp, 0, 0, 0, 0.5);
    commandSetSetpoint(&_sp);
}

static void commandLand() {
    setpoint_t _sp;
    _sp.timestamp = 0;
    _sp.duration = 1000;
    getHoverSetpoint(&_sp, 0, 0, 0, 0.2);
    commandSetSetpoint(&_sp);

    _sp.duration = 1000;
    getRpytSetpoint(&_sp, 0, 0, 0, 11200);
    commandSetSetpoint(&_sp);

    // indicate landing
    _sp.duration = 0;
    _sp.thrust = -1.0f;
    commandSetSetpoint(&_sp);

    // set to landing state and reject new commands
    commandState = COMMAND_STATE_LANDING;
}

void commandInit(void) {
    STATIC_QUEUE_INIT(commandQueue);
}

void commandGetSetpoint(setpoint_t *s) {
    if (commandState == COMMAND_STATE_NORMAL && supervisorCommandTimeout()) {
        // clear the queue
        while (STATIC_QUEUE_RECEIVE(commandQueue, &currentSetpoint, 0) == osOK) {}
        commandLand();
    }

    // check if the setpoint is expired
    if (currentSetpoint.timestamp + currentSetpoint.duration < osKernelGetTickCount()
        && !STATIC_QUEUE_IS_EMPTY(commandQueue)) {
        STATIC_QUEUE_RECEIVE(commandQueue, &currentSetpoint, 0);
        currentSetpoint.timestamp = osKernelGetTickCount();
    }

    if (currentSetpoint.thrust < 0) {
        commandState = COMMAND_STATE_LANDED;

        // clear the queue
        while (STATIC_QUEUE_RECEIVE(commandQueue, &currentSetpoint, 0) == osOK) {}
        // clear the setpoint
        getRpytSetpoint(&currentSetpoint, 0, 0, 0, 0);
    }
    
    *s = currentSetpoint;
}

void commandSetSetpoint(setpoint_t *s) {
    // do not accept setpoint when the it's not in normal mode
    if (commandState != COMMAND_STATE_NORMAL) {
        return;
    }
    STATIC_QUEUE_SEND(commandQueue, s, 0);
}

void commandProcessPacket(PodtpPacket *packet) {
    static setpoint_t sp;
    memset(&sp, 0, sizeof(setpoint_t));
    switch (packet->port) {
        case PORT_COMMAND_RPYT:
            commandDecodeRpytPacket(packet, &sp);
            break;
        case PORT_COMMAND_TAKEOFF:
            commandTakeOff();
            break;
        case PORT_COMMAND_LAND:
            commandLand();
            break;
        case PORT_COMMAND_HOVER:
            commandDecodeHoverPacket(packet, &sp);
            break;
        case PORT_COMMAND_POSITION:
            commandDecodeXyzyPacket(packet, &sp);
            break;
        case PORT_COMMAND_VELOCITY:
            commandDecodeVelocityPacket(packet, &sp);
            break;
        default:
            // unknown command
            packet->port = PORT_ACK_ERROR;
            return;
    }

    packet->port = PORT_ACK_OK;
}

command_state_t commandGetState(void) {
    return commandState;
}

void commandSetState(command_state_t state) {
    commandState = state;
}