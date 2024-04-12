#include "grid_tof.h"
#include "_config.h"
#include "_i2c.h"
#include "debug.h"
#include "vl53l8cx_api.h"
#include "utils.h"

VL53L8CX_Configuration vl53l8Dev;

uint32_t gridTofInit(void) {
    uint8_t status, isAlive;

    vl53l8Dev.platform.read = vl53l8cx_read_dma;
    vl53l8Dev.platform.write = vl53l8cx_write_normal;
    vl53l8Dev.platform.delay = sensorsDelayMs;
    vl53l8Dev.platform.millis = sensorsGetMilli;
    vl53l8Dev.platform.address = VL53L8CX_DEFAULT_I2C_ADDRESS;

    status = vl53l8cx_is_alive(&vl53l8Dev, &isAlive);
    if (!isAlive || status) {
		DEBUG_PRINT("VL53L8CX not found\n");
		return status;
	}

	/* (Mandatory) Init VL53L8CX sensor */
	status = vl53l8cx_init(&vl53l8Dev);
	if (status) {
		DEBUG_PRINT("VL53L8CX Init [FAILED]\n");
		return status;
	} else {
        DEBUG_PRINT("VL53L8CX Init [OK]\n");
    }
    return 0;
}