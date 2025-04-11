#define MODULE_NAME "DIS"
#include "distance.h"
#include "_config.h"
#include "_i2c.h"
#include "debug.h"
#include "vl53l8cx_api.h"
#include "utils.h"
#include "stabilizer_types.h"
#include "system.h"

// 3 for car, 1 for drone
#define DIS_SENSOR_COUNT 3

VL53L8CX_Configuration vl53l8Dev[DIS_SENSOR_COUNT];
STATIC_TASK_DEF(distanceTask, DIS_TASK_PRIORITY, DIS_TASK_STACK_SIZE);

#define DIS_TASK_RATE 14
#define RESOLUTION VL53L8CX_RESOLUTION_8X8

volatile uint8_t data __attribute__((section(".ramd3"), aligned(4))) = 0;

static bool pca9546Init(void) {
    if (HAL_I2C_IsDeviceReady(&VL53L8CX_I2C_HANDLE, 0x70 << 1, 3, 1000) != HAL_OK) {
        DEBUG_PRINT("PCA9546 not found\n");
        return false;
    }
    return true;
}

static void selectChannel(uint8_t channel) {
    uint8_t data = 1 << channel;
    HAL_I2C_Master_Transmit(&VL53L8CX_I2C_HANDLE, 0x70 << 1, &data, 1, 1000);
}

typedef struct {
    uint32_t timestamp;
    int16_t distance[RESOLUTION];
} distance_t;

uint32_t distanceInit(void) {
    if (!pca9546Init()) {
        return TASK_INIT_FAILED(DIS_TASK_INDEX);
    }
    uint8_t status = 0, isAlive;

    for (int i = 0; i < DIS_SENSOR_COUNT; i++) {
        selectChannel(i);
        vl53l8Dev[i].platform.read = vl53l8cx_read_dma;
        vl53l8Dev[i].platform.write = vl53l8cx_write_normal;
        vl53l8Dev[i].platform.delay = sensorsDelayMs;
        vl53l8Dev[i].platform.millis = sensorsGetMilli;
        vl53l8Dev[i].platform.address = VL53L8CX_DEFAULT_I2C_ADDRESS >> 1;

        status = vl53l8cx_is_alive(&vl53l8Dev[i], &isAlive);
        if (!isAlive || status) {
            DEBUG_PRINT("VL53L8CX[%d] not found\n", i);
            return status;
        }

        /* (Mandatory) Init VL53L8CX sensor */
        status = vl53l8cx_init(&vl53l8Dev[i]);
        if (status) {
            DEBUG_PRINT("VL53L8CX[%d] Init [FAILED]\n", i);
            return status;
        }
    }

    if (status == 0 && DIS_SENSOR_COUNT > 0) {
        DEBUG_PRINT("VL53L8CX Init [OK] x %d\n", DIS_SENSOR_COUNT);
        STATIC_TASK_INIT(distanceTask, NULL);
    }
    return status;
}

static int16_t front[8], left[8], right[8];
void distanceTask(void *argument) {
    VL53L8CX_ResultsData rangingData[DIS_SENSOR_COUNT];
    uint8_t isReady, status;
    systemWaitStart();
    DEBUG_PRINT("Distance Task [START]\n");
    for (int i = 0; i < DIS_SENSOR_COUNT; i++) {
        selectChannel(i);
        status = vl53l8cx_set_ranging_mode(&vl53l8Dev[i], VL53L8CX_RANGING_MODE_CONTINUOUS);
        status |= vl53l8cx_set_resolution(&vl53l8Dev[i], RESOLUTION);
        status |= vl53l8cx_set_ranging_frequency_hz(&vl53l8Dev[i], 15);
        if (status) {
            DEBUG_REMOTE("VL53L8CX[%d] Config Ranging [FAILED]: %u\n", i, status);
            return;
        }
        status = vl53l8cx_start_ranging(&vl53l8Dev[i]);
        if (status) {
            DEBUG_REMOTE("VL53L8CX[%d] Start Ranging [FAILED]: %u\n", i, status);
            return;
        }
    }
    
    TASK_TIMER_DEF(DIS, DIS_TASK_RATE);

    distance_t distanceBuffer;
    while (1) {
        TASK_TIMER_WAIT(DIS);
        for (int i = 0; i < DIS_SENSOR_COUNT; i++) {
            selectChannel(i);
            vl53l8cx_check_data_ready(&vl53l8Dev[i], &isReady);
            if (isReady) {
                status = vl53l8cx_get_ranging_data(&vl53l8Dev[i], &rangingData[i]);
                if (status != 0) {
                    DEBUG_REMOTE("VL53L8CX[%d] Ranging [FAILED]: %d\n", i, status);
                    continue;
                }

                if (i == 0) {
                    for (int j = 0; j < RESOLUTION; j++) {
                        uint8_t status = rangingData[i].target_status[VL53L8CX_NB_TARGET_PER_ZONE * j];
                        int16_t distance = rangingData[i].distance_mm[VL53L8CX_NB_TARGET_PER_ZONE * j];
                        distanceBuffer.distance[j] = 
                            (distance & 0x7FFF) | ((status != 5 && status != 9) << 15);
                    }
                }

                for (int j = 0; j < 8; j++) {
                    uint8_t index = 3 * 8 + j;
                    uint8_t status = rangingData[i].target_status[VL53L8CX_NB_TARGET_PER_ZONE * index];
                    int16_t distance = rangingData[i].distance_mm[VL53L8CX_NB_TARGET_PER_ZONE * index];
                    switch (i) {
                        case 0:
                            front[j] = (distance & 0x7FFF) | ((status != 5 && status != 9) << 15);
                            break;
                        case 1:
                            left[j] = (distance & 0x7FFF) | ((status != 5 && status != 9) << 15);
                            break;
                        case 2:
                            right[j] = (distance & 0x7FFF) | ((status != 5 && status != 9) << 15);
                            break;
                    }
                }
            } else {
                DEBUG_REMOTE("VL53L8CX Data not ready\n");
            }
        }
        distanceBuffer.timestamp = osKernelGetTickCount();
        if (DIS_SENSOR_COUNT > 1) {
            for (int i = 0; i < 8; i++) {
                distanceBuffer.distance[i] = left[i];
                distanceBuffer.distance[i + 8 * 7] = right[i];
            }
        }

        if (DIS_SENSOR_COUNT > 0) {
            linkSendData(PODTP_TYPE_LOG, PORT_LOG_DISTANCE, (uint8_t *)&distanceBuffer, sizeof(distance_t));
        }
    }
}

#define LONG_OBSTACLE_DIST 1500
#define SHORT_OBSTACLE_DIST 500
#define CRIT_OBSTACLE_DIST 300
#define NOISE_LEVEL 200
static bool canMove(int16_t *dist, int size) {
    for (int i = 0; i < size; i++) {
        if (dist[i] > 0 && dist[i] < SHORT_OBSTACLE_DIST) return false;
    }
    return true;
}

static bool canFreelyMove(int16_t *dist, int size) {
    for (int i = 0; i < size; i++) {
        if (dist[i] > 0 && dist[i] < LONG_OBSTACLE_DIST) return false;
    }
    return true;
}

void distanceAdjustSpeed(float current_vx, float *vx, float *vy) {
    int valid_count = 0;
    // only apply to moving forward command
    if (*vy != 0 || *vx <= 0) return;

    // if central front is clear, just go
    if (canFreelyMove(&front[2], 4)) {
        return;
    }

    // if central front is closely blocked, stop
    if (!canMove(&front[2], 4)) {
        float front_ave = 0;
        valid_count = 0;
        for (int i = 0; i < 4; i++) {
            if (front[i + 2] > 0) {
                valid_count++;
                front_ave += front[i + 2];
            }
        }

        if (valid_count == 0) {
            *vx = 0;  // Stop
            return;
        }

        front_ave /= valid_count;

        // DEBUG_REMOTE("Obstacle in front: %.1f\n", front_ave);

        if (front_ave < CRIT_OBSTACLE_DIST) {
            *vx = -*vx * 0.5;  // Move backward
            return;
        }

        if (front_ave >= CRIT_OBSTACLE_DIST && front_ave < SHORT_OBSTACLE_DIST && current_vx < 0.2) {
            *vx = 0;  // Stop
            return;
        }

        *vx = -*vx * 0.5;  // Move backward
        return;
    }

    // there is obstacle in the front between LONG_OBSTACLE_DIST and SHORT_OBSTACLE_DIST
    // TODO: test sensor
    float front_l_sum = 0, front_r_sum = 0;
    for (int i = 0; i < 4; i++) {
        if (front[i] > 0) {
            valid_count++;
            front_l_sum += front[i];
        }
    }
    front_l_sum /= valid_count + 1;
    valid_count = 0;
    for (int i = 0; i < 4; i++) {
        if (front[4 + i] > 0) {
            valid_count++;
            front_r_sum += front[4 + i];
        }
    }
    front_r_sum /= valid_count + 1;

    // front left/right is clear for move
    if (front_l_sum < front_r_sum - NOISE_LEVEL) {
        if (canMove(&right[0], 5)) {
            *vy = -0.5 * (*vx);
        }
        return;
    } else if (front_r_sum < front_l_sum - NOISE_LEVEL) {
        if (canMove(&left[3], 5)) {
            *vy = 0.5 * (*vx);
        }
        return;
    }
}