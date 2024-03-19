#ifndef __VL53L1_H__
#define __VL53L1_H__

#include <stdbool.h>
#include <stdint.h>
#include "vl53l1_platform.h"
#include "vl53l1_api.h"
#include "vl53l1_register_settings.h"

#define VL53L1_DEV_ADDR_DEFAULT 0x29
int8_t vl53l1Init(VL53L1_Dev_t *dev);
bool vl53l1Test(VL53L1_Dev_t *dev);

#endif