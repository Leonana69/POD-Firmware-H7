#define MODULE_NAME "DIS"
#include "distance.h"
#include "_config.h"
#include "_i2c.h"
#include "debug.h"
#include "vl53l8cx_api.h"
#include "utils.h"
#include "stabilizer_types.h"
#include "system.h"

VL53L8CX_Configuration vl53l8Dev;
STATIC_TASK_DEF(distanceTask, DIS_TASK_PRIORITY, DIS_TASK_STACK_SIZE);

#define DIS_TASK_RATE 1
// #define DIS_TASK_RATE RATE_25_HZ

volatile uint8_t data __attribute__((section(".ramd3"), aligned(4))) = 0;

bool pca9546Init(void) {
    if (HAL_I2C_IsDeviceReady(&VL53L8CX_I2C_HANDLE, 0x70 << 1, 3, 1000) != HAL_OK) {
        DEBUG_PRINT("PCA9546 not found\n");
        return false;
    }
    return true;
}

void enableChannel(uint8_t channel) {
    uint8_t data = 1 << channel;
    HAL_I2C_Master_Transmit(&VL53L8CX_I2C_HANDLE, 0x70 << 1, &data, 1, 1000);
}

uint32_t distanceInit(void) {
    if (!pca9546Init()) {
        return TASK_INIT_FAILED(DIS_TASK_INDEX);
    }
    enableChannel(0);
    uint8_t status, isAlive;

    vl53l8Dev.platform.read = vl53l8cx_read_dma;
    vl53l8Dev.platform.write = vl53l8cx_write_normal;
    vl53l8Dev.platform.delay = sensorsDelayMs;
    vl53l8Dev.platform.millis = sensorsGetMilli;
    vl53l8Dev.platform.address = VL53L8CX_DEFAULT_I2C_ADDRESS >> 1;

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
        STATIC_TASK_INIT(distanceTask, NULL);
    }
    return 0;
}

void distanceTask(void *argument) {
    VL53L8CX_ResultsData rangingData;
    uint8_t isReady, status;
    systemWaitStart();
    status = vl53l8cx_set_ranging_mode(&vl53l8Dev, VL53L8CX_RANGING_MODE_CONTINUOUS);
    status |= vl53l8cx_set_resolution(&vl53l8Dev, VL53L8CX_RESOLUTION_8X8);
    status |= vl53l8cx_set_ranging_frequency_hz(&vl53l8Dev, 15);

    if (status) {
        DEBUG_PRINT("VL53L8CX Config Ranging [FAILED]: %u\n", status);
		return;
    }

    status = vl53l8cx_start_ranging(&vl53l8Dev);
    if (status) {
        DEBUG_PRINT("VL53L8CX Start Ranging [FAILED]: %u\n", status);
        return;
    }

    
    TASK_TIMER_DEF(DIS, DIS_TASK_RATE);
    while (1) {
        TASK_TIMER_WAIT(DIS);
        vl53l8cx_check_data_ready(&vl53l8Dev, &isReady);
        if (isReady) {
            status = vl53l8cx_get_ranging_data(&vl53l8Dev, &rangingData);
            if (status == 0) {
                // for (int i = 0; i < 8; i++) {
                //     for (int j = 0; j < 8; j++) {
                //         DEBUG_PRINT("%4d\t",
                //         rangingData.distance_mm[VL53L8CX_NB_TARGET_PER_ZONE * i]);
                //     }
                //     DEBUG_PRINT("\n");
                // }
            } else {
                DEBUG_PRINT("VL53L8CX Ranging [FAILED]: %d\n", status);
            }
        } else {
            DEBUG_PRINT("VL53L8CX Data not ready\n");
        }
    }
}