#ifndef __PAA3905_H__
#define __PAA3905_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

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

struct paa3905_dev {
    uint8_t chip_id;
    paa3905_read_t read;
    paa3905_write_t write;
    paa3905_delay_t delay;
    uint8_t mode;
};

int8_t paa3905_init(struct paa3905_dev *dev);

#ifdef __cplusplus
}
#endif

#endif //__PAA3905_H__