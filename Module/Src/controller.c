#include "controller.h"
#include "controller_drone.h"
#include "controller_gear.h"

typedef struct {
    void (*init)();
    void (*update)(
        setpoint_t *setpoint, imu_t *imu, state_t *state,
        uint32_t tick, control_t *control_out);
} controller_t;

#ifdef GEAR
static controller_t controller = {
    .init = controllerGearInit,
    .update = controllerGearUpdate,
};
#else
static controller_t controller = {
    .init = controllerDroneInit,
    .update = controllerDroneUpdate,
};
#endif

void controllerInit() {
    controller.init();
}

void controllerUpdate(
    setpoint_t *setpoint, imu_t *imu, state_t *state,
    uint32_t tick, control_t *control_out) {
    controller.update(setpoint, imu, state, tick, control_out);
}