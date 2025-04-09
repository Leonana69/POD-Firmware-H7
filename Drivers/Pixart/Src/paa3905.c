#include "paa3905.h"
#include "_spi.h"

#include "debug.h"

uint8_t set_mode_reg_addr[60] = {
    0x7F, 0x51, 0x4E, 0x66,
    0x7F, 0x7E, 0x55, 0x59,
    0x6F, 0x7F, 0x4D, 0x4E,
    0x7F, 0x5C, 0x5F, 0x70,
    0x71, 0x72, 0x74, 0x75,
    0x76, 0x4E, 0x7F, 0x64,
    0x65, 0x66, 0x55, 0x56,
    0x57, 0x4A, 0x4B, 0x4E,
    0x69, 0x7F, 0x69, 0x47,
    0x48, 0x5A, 0x75, 0x4A,
    0x42, 0x45, 0x65, 0x67,
    0x68, 0x6A, 0x43, 0x7F,
    0x4A, 0x4B, 0x4C, 0x4D,
    0x46, 0x59, 0x7F, 0x4A,
    0x48, 0x52, 0x7F, 0x5B,
};

uint8_t standard_mode_reg_data[60] = {
    0x00, 0xFF, 0x2A, 0x3E,
    0x14, 0x71, 0x00, 0x00,
    0x2C, 0x05, 0xAC, 0x32,
    0x09, 0xAF, 0xAF, 0x08,
    0x04, 0x06, 0x3C, 0x28,
    0x20, 0xBF, 0x03, 0x14,
    0x0A, 0x10, 0x3C, 0x28,
    0x20, 0x2D, 0x2D, 0x4B,
    0xFA, 0x05, 0x1F, 0x1F,
    0x0C, 0x20, 0x0F, 0x0F,
    0x02, 0x03, 0x00, 0x76,
    0x76, 0xC5, 0x00, 0x06,
    0x18, 0x0C, 0x0C, 0x0C,
    0x0A, 0xCD, 0x0A, 0x2A,
    0x96, 0xB4, 0x00, 0xA0,
};

uint8_t enhanced_mode_reg_data[60] = {
    0x00, 0xFF, 0x2A, 0x26,
    0x14, 0x71, 0x00, 0x00,
    0x2C, 0x05, 0xAC, 0x65,
    0x09, 0xAF, 0xAF, 0x00,
    0x00, 0x00, 0x14, 0x14,
    0x06, 0x8F, 0x03, 0x00,
    0x00, 0x00, 0x14, 0x14,
    0x06, 0x20, 0x20, 0x32,
    0xFE, 0x05, 0x14, 0x14,
    0x1C, 0x20, 0xE5, 0x05,
    0x04, 0x03, 0x00, 0x50,
    0x50, 0xC5, 0x00, 0x06,
    0x1E, 0x1E, 0x34, 0x34,
    0x32, 0x0D, 0x0A, 0x2A,
    0x96, 0xB4, 0x00, 0xA0,
};

void register_write(paa3905_dev_t *dev, uint8_t reg_addr, uint8_t reg_data) {
    dev->write(reg_addr, &reg_data, 1, NULL);
}

uint8_t register_read(paa3905_dev_t *dev, uint8_t reg_addr) {
    uint8_t reg_data;
    dev->read(reg_addr, &reg_data, 1, NULL);
    return reg_data;
}

void paa3905_standard_mode(paa3905_dev_t *dev) {
    for (int i = 0; i < 60; i++) {
        register_write(dev, set_mode_reg_addr[i], standard_mode_reg_data[i]);
    }
}

void paa3905_enhanced_mode(paa3905_dev_t *dev) {
    for (int i = 0; i < 60; i++) {
        register_write(dev, set_mode_reg_addr[i], enhanced_mode_reg_data[i]);
    }
}

void paa3905_disable_motion_cutoff(paa3905_dev_t *dev) {
    register_write(dev, 0x7F, 0x05);
    register_write(dev, 0x6A, 0xFF);
    register_write(dev, 0x6B, 0x3F);
    register_write(dev, 0x7F, 0x00);
}

void paa3905_set_auto_switch(paa3905_dev_t *dev, bool mode2) {
    register_write(dev, 0x7F, 0x08);
    register_write(dev, 0x68, mode2 ? 0x02 : 0x01);
    register_write(dev, 0x7F, 0x00);
}

int8_t paa3905_init(paa3905_dev_t *dev) {
    dev->delay(40);
    uint8_t chip_id = register_read(dev, PAA3905_REG_PRODUCT_ID);
    uint8_t inverse_chip_id = register_read(dev, PAA3905_REG_INVERSE_PRODUCT_ID);

    if (chip_id != PAA3905_PRODUCT_ID || inverse_chip_id != PAA3905_INVERSE_PRODUCT_ID) {
        return PAA3905_ERROR;
    }

    register_write(dev, PAA3905_REG_POWER_UP_RESET, 0x5A);
    dev->delay(2);
    register_read(dev, PAA3905_REG_MOTION);
    register_read(dev, PAA3905_REG_DELTA_X_L);
    register_read(dev, PAA3905_REG_DELTA_X_H);
    register_read(dev, PAA3905_REG_DELTA_Y_L);
    register_read(dev, PAA3905_REG_DELTA_Y_H);

    if (dev->mode == PAA3905_ENHANCED_MODE)
        paa3905_enhanced_mode(dev);
    else
        paa3905_standard_mode(dev);

    dev->delay(2);
    paa3905_set_auto_switch(dev, false);
    return PAA3905_OK;
}

void paa3905_motion_burst(paa3905_dev_t *dev, paa3905_motion_t *motion) {
    dev->read(PAA3905_REG_MOTION_BURST, (uint8_t *)motion, sizeof(paa3905_motion_t), NULL);
}
