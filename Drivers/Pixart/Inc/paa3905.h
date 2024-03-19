#ifndef __PAA3905_H__
#define __PAA3905_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#define PAA3905_OK    0
#define PAA3905_ERROR -1

#define PAA3905_PRODUCT_ID 0xA2
#define PAA3905_REVISION_ID 0x00
#define PAA3905_RESOLUTION 0x2A
#define PAA3905_ORIENTATION 0xE0
#define PAA3905_INVERSE_PRODUCT_ID 0x5D

#define PAA3905_REG_PRODUCT_ID 0x00
#define PAA3905_REG_REVISION_ID 0x01
#define PAA3905_REG_MOTION 0x02
#define PAA3905_REG_DELTA_X_L 0x03
#define PAA3905_REG_DELTA_X_H 0x04
#define PAA3905_REG_DELTA_Y_L 0x05
#define PAA3905_REG_DELTA_Y_H 0x06
#define PAA3905_REG_SQUAL 0x07
#define PAA3905_REG_RAW_DATA_SUM 0x08
#define PAA3905_REG_MAXIMUM_RAW_DATA 0x09
#define PAA3905_REG_MINIMUM_RAW_DATA 0x0A
#define PAA3905_REG_SHUTTER_LOWER 0x0B
#define PAA3905_REG_SHUTTER_MIDDLE 0x0C
#define PAA3905_REG_SHUTTER_UPPER 0x0D
#define PAA3905_REG_RAW_DATA_GRUB_STATUS 0x10
#define PAA3905_REG_RAW_DATA_GRUB 0x13
#define PAA3905_REG_OBSERVATION 0x15
#define PAA3905_REG_MOTION_BURST 0x16
#define PAA3905_REG_POWER_UP_RESET 0x3A
#define PAA3905_REG_SHUTDOWN 0x3B
#define PAA3905_REG_RESOULTION 0x4E
#define PAA3905_REG_ORIENTATION 0x5B
#define PAA3905_REG_INVERSE_PRODUCT_ID 0x5F

#define PAA3905_STANDARD_MODE 0x00
#define PAA3905_ENHANCED_MODE 0x01

// this definition is consistent with Bosch BMI270
typedef int8_t (*paa3905_read_t)(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
typedef int8_t (*paa3905_write_t)(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
typedef void (*paa3905_delay_t)(uint32_t period);

typedef struct {
    paa3905_read_t read;
    paa3905_write_t write;
    paa3905_delay_t delay;
    uint8_t mode;
}paa3905_dev_t;

typedef struct {
    uint8_t motion;
    uint8_t observation;
    union {
        struct {
            uint8_t delta_x_l;
            uint8_t delta_x_h;
        };
        int16_t delta_x;
    };
    union {
        struct {
            uint8_t delta_y_l;
            uint8_t delta_y_h;
        };
        int16_t delta_y;
    };
    uint8_t challenging_condition;
    uint8_t squal;
    uint8_t raw_data_sum;
    uint8_t maximum_raw_data;
    uint8_t minimum_raw_data;
    uint8_t shutter_upper;
    uint8_t shutter_middle;
    uint8_t shutter_lower;
} paa3905_motion_t;

int8_t paa3905_init(paa3905_dev_t *dev);
void paa3905_motion_burst(paa3905_dev_t *dev, paa3905_motion_t *motion);

#ifdef __cplusplus
}
#endif

#endif //__PAA3905_H__