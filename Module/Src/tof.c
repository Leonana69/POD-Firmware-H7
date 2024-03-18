#define MODULE_NAME "TOF"
#include "debug.h"

#include "tof.h"
#include "_i2c.h"
#include "_config.h"
#include "config.h"
#include "vl53l1.h"
#include "freeRTOS_helper.h"
#include "utils.h"
#include "system.h"
#include "stabilizer_types.h"
#include "estimator_kalman.h"

STATIC_TASK_DEF(tofTask, TOF_TASK_PRIORITY, TOF_TASK_STACK_SIZE);

#define TOF_RATE RATE_25_HZ
static VL53L1_Dev_t vl53l1Dev;
static tof_t tofData;

uint32_t tofInit(void) {
    if (HAL_I2C_IsDeviceReady(&VL53L1_I2C_HANDLE, VL53L1_DEV_ADDR_DEFAULT << 1, 3, 100) != HAL_OK) {
        DEBUG_PRINT("VL53L1 not found\n");
        return TASK_INIT_FAILED(TOF_TASK_INDEX);
    }

    vl53l1Dev.read = vl53l1_read_dma;
    vl53l1Dev.write = vl53l1_write_normal;
    vl53l1Dev.delay = sensorsDelayMs;
    vl53l1Dev.millis = sensorsGetMilli;
    vl53l1Dev.i2c_slave_address = VL53L1_DEV_ADDR_DEFAULT;

    int8_t status = vl53l1Init(&vl53l1Dev);
    if (status != VL53L1_ERROR_NONE) {
        DEBUG_PRINT("VL53L1 Init [FAILED]: %d\n", status);
        return TASK_INIT_FAILED(TOF_TASK_INDEX);
    } else {
        DEBUG_PRINT("VL53L1 Init [OK]\n");
        STATIC_TASK_INIT(tofTask, NULL);
    }

    return TASK_INIT_SUCCESS;
}

void tofTask(void *argument) {
    estimatorPacket_t packet = { .type = TOF_TASK_INDEX };
    systemWaitStart();
    TASK_TIMER_DEF(TOF, TOF_RATE);
	VL53L1_RangingMeasurementData_t vl53l1RangingData;

	int16_t range_mm = 0;
	while (1) {
		TASK_TIMER_WAIT(TOF);
		VL53L1_WaitMeasurementDataReady(&vl53l1Dev);
		VL53L1_GetRangingMeasurementData(&vl53l1Dev, &vl53l1RangingData);
		range_mm = vl53l1RangingData.RangeMilliMeter;
		VL53L1_ClearInterruptAndStartMeasurement(&vl53l1Dev);

		if (range_mm < RANGE_OUTLIER_LIMIT) {
			tofData.distance = range_mm * 0.001f;
			tofData.stdDev = expStdA * (1.0f  + expf(expCoeff * (range_mm * 0.001f - expPointA)));
			packet.tof = tofData;
            estimatorKalmanEnqueue(&packet);
		}
	}
}