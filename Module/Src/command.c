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
static podtp_error_type commandSetSetpoint(setpoint_t *s);

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
static podtp_error_type commandDecodeHoverPacket(const PodtpPacket *packet, setpoint_t *sp) {
    if (packet->length - 1 != sizeof(hover_t)) {
        return PODTP_ERROR_SIZE;
    }
    hover_t *hover = (hover_t *)packet->data;
    getHoverSetpoint(sp, hover->vx, hover->vy, hover->vyaw, hover->height);
    return commandSetSetpoint(sp);
}

typedef struct {
    scalar_t vx;
    scalar_t vy;
    scalar_t vyaw;
    scalar_t vz;
} __attribute__((packed)) vxyzy_t;
static podtp_error_type commandDecodeVelocityPacket(const PodtpPacket *packet, setpoint_t *sp) {
    if (packet->length - 1 != sizeof(vxyzy_t)) {
        return PODTP_ERROR_SIZE;
    }
    vxyzy_t *vxyzy = (vxyzy_t *)packet->data;
    getVelocitySetpoint(sp, vxyzy->vx, vxyzy->vy, vxyzy->vyaw, vxyzy->vz);
    return commandSetSetpoint(sp);
}

typedef struct {
    scalar_t roll;
    scalar_t pitch;
    scalar_t yaw;
    scalar_t thrust;
} __attribute__((packed)) rpyt_t;
static podtp_error_type commandDecodeRpytPacket(const PodtpPacket *packet, setpoint_t *sp) {
    if (packet->length - 1 != sizeof(rpyt_t)) {
        return PODTP_ERROR_SIZE;
    }
    rpyt_t *rpyt = (rpyt_t *)packet->data;
    getRpytSetpoint(sp, rpyt->roll, rpyt->pitch, rpyt->yaw, rpyt->thrust);
    return commandSetSetpoint(sp);
}

typedef struct {
    scalar_t x;
    scalar_t y;
    scalar_t z;
    scalar_t yaw;
} __attribute__((packed)) xyzy_t;
static podtp_error_type commandDecodeXyzyPacket(const PodtpPacket *packet, setpoint_t *sp) {
    if (packet->length - 1 != sizeof(rpyt_t)) {
        return PODTP_ERROR_SIZE;
    }
    xyzy_t *xyzy = (xyzy_t *)packet->data;
    getXyzySetpoint(sp, xyzy->x, xyzy->y, xyzy->z, xyzy->yaw);
    return commandSetSetpoint(sp);
}

static podtp_error_type commandTakeOff() {
    podtp_error_type ret = PODTP_ERROR_NONE;
    setpoint_t _sp;
    _sp.timestamp = 0;
    _sp.duration = 500;
    getRpytSetpoint(&_sp, 0, 0, 0, 11600);
    ret |= commandSetSetpoint(&_sp);

    _sp.duration = 1000;
    getRpytSetpoint(&_sp, 0, 0, 0, 12000);
    ret |= commandSetSetpoint(&_sp);

    // keep hovering
    _sp.duration = 0;
    getHoverSetpoint(&_sp, 0, 0, 0, 0.5);
    ret |= commandSetSetpoint(&_sp);
    commandState = COMMAND_STATE_HOVERING;

    return ret;
}

static podtp_error_type commandLand() {
    podtp_error_type ret = PODTP_ERROR_NONE;
    setpoint_t _sp;
    _sp.timestamp = 0;
    _sp.duration = 1000;
    getHoverSetpoint(&_sp, 0, 0, 0, 0.2);
    ret |= commandSetSetpoint(&_sp);

    _sp.duration = 1000;
    getRpytSetpoint(&_sp, 0, 0, 0, 11200);
    ret |= commandSetSetpoint(&_sp);

    // indicate landing
    _sp.duration = 0;
    _sp.thrust = -1.0f;
    ret |= commandSetSetpoint(&_sp);

    // set to landing state and reject new commands
    commandState = COMMAND_STATE_LANDING;
    return ret;
}

void commandInit(void) {
    STATIC_QUEUE_INIT(commandQueue);
}

void commandGetSetpoint(setpoint_t *s) {
    if (commandState != COMMAND_STATE_LANDING
    && commandState != COMMAND_STATE_LANDED
    && supervisorCommandTimeout()) {
        // clear the queue
        while (STATIC_QUEUE_RECEIVE(commandQueue, &currentSetpoint, 0) == osOK) {}
        if (commandState == COMMAND_STATE_HOVERING) {
            // do landing
            commandLand();
        }
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

podtp_error_type commandSetSetpoint(setpoint_t *s) {
    // do not accept setpoint when the it's not in normal or hovering state
    if (commandState != COMMAND_STATE_NORMAL
     && commandState != COMMAND_STATE_HOVERING) {
        return PODTP_ERROR_LOCKED;
    }
    STATIC_QUEUE_SEND(commandQueue, s, 0);
    return PODTP_ERROR_NONE;
}

podtp_error_type commandProcessPacket(const PodtpPacket *packet) {
    static setpoint_t sp;
    podtp_error_type ret;
    memset(&sp, 0, sizeof(setpoint_t));
    switch (packet->port) {
        case PORT_COMMAND_RPYT:
            ret = commandDecodeRpytPacket(packet, &sp);
            if (ret == PODTP_ERROR_NONE) {
                commandState = COMMAND_STATE_NORMAL;
            }
            break;
        case PORT_COMMAND_TAKEOFF:
            ret = commandTakeOff();
            break;
        case PORT_COMMAND_LAND:
            ret = commandLand();
            break;
        case PORT_COMMAND_HOVER:
            ret = commandDecodeHoverPacket(packet, &sp);
            if (ret == PODTP_ERROR_NONE) {
                commandState = COMMAND_STATE_HOVERING;
            }
            break;
        case PORT_COMMAND_POSITION:
            ret = commandDecodeXyzyPacket(packet, &sp);
            if (ret == PODTP_ERROR_NONE) {
                commandState = COMMAND_STATE_NORMAL;
            }
            break;
        case PORT_COMMAND_VELOCITY:
            ret = commandDecodeVelocityPacket(packet, &sp);
            if (ret == PODTP_ERROR_NONE) {
                commandState = COMMAND_STATE_NORMAL;
            }
            break;
        default:
            // unknown command
            ret = PODTP_ERROR_UNKNOWN_COMMAND;
    }

    return ret;
}

command_state_t commandGetState(void) {
    return commandState;
}

void commandSetState(command_state_t state) {
    commandState = state;
}