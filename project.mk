OPENOCD				?= openocd
OPENOCD_INTERFACE	?= interface/stlink.cfg
OPENOCD_TARGET    	?= target/stm32h7x.cfg
OPENOCD_CMDS		?=
LOAD_ADDRESS		?= 0x8020000
PROG				?= $(BUILD_DIR)/$(TARGET)

######################################
# POD's C includes
######################################
C_INCLUDES += \
-IDrivers/Bosch/Inc \
-IDrivers/Vl53l1/Inc \
-IDrivers/Vl53l1_Platform/Inc \
-IHAL/Inc \
-IModule/Inc

VPATH += Module/Src
C_SOURCES += eprintf.c

VPATH += HAL/Src
C_SOURCES += _usart.c

VPATH += Drivers/Bosch/Src
C_SOURCES += bmi08a.c bmi08g.c bmp3.c bmi2.c bmi270.c

VPATH += Drivers/Vl53l1/Src
C_SOURCES += vl53l1_api_calibration.c vl53l1_api_core.c vl53l1_api_debug.c vl53l1_api_preset_modes.c vl53l1_api_strings.c \
	vl53l1_api.c vl53l1_core_support.c vl53l1_core.c vl53l1_error_strings.c vl53l1_register_funcs.c vl53l1_silicon_core.c \
	vl53l1_wait.c vl53l1.c

all: $(PROG).bin

copy:
	@echo "Copy $(PROG).bin to scripts folder"
	@rm -f /Users/guojun/Workspace/github/POD-XIAO-Firmware/scripts/$(PROG).bin
	@cp $(PROG).bin /Users/guojun/Workspace/github/POD-XIAO-Firmware/scripts/

flash:
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) $(OPENOCD_CMDS) -f $(OPENOCD_TARGET) -c init -c targets -c "reset halt" \
                 -c "flash write_image erase $(PROG).bin $(LOAD_ADDRESS) bin" -c "reset run" -c shutdown