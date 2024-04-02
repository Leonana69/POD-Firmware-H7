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
-IDrivers/CMSIS/DSP/Include \
-IDrivers/Bosch/Inc \
-IDrivers/Pixart/Inc \
-IDrivers/Vl53l1/Inc \
-IDrivers/Vl53l1_Platform/Inc \
-IHAL/Inc \
-IModule/Inc

C_DEFS += -DGEAR

CFLAGS += -Wno-comment

C_SOURCES += $(wildcard Module/Src/*.c)
C_SOURCES += $(wildcard HAL/Src/*.c)
C_SOURCES += $(wildcard Drivers/Bosch/Src/*.c)
C_SOURCES += $(wildcard Drivers/Pixart/Src/*.c)
C_SOURCES += $(wildcard Drivers/Vl53l1/Src/*.c)

VPATH += Drivers/CMSIS/DSP/Source/FastMathFunctions
C_SOURCES += arm_cos_f32.c arm_sin_f32.c

VPATH += Drivers/CMSIS/DSP/Source/MatrixFunctions
C_SOURCES += arm_mat_mult_f32.c arm_mat_trans_f32.c

VPATH += Drivers/CMSIS/DSP/Source/CommonTables
C_SOURCES += arm_common_tables.c

all: $(PROG).bin

copy:
	@echo "Copy $(PROG).bin to scripts folder"
	@rm -f /Users/guojun/Workspace/github/POD-XIAO-Firmware/scripts/$(PROG).bin
	@cp $(PROG).bin /Users/guojun/Workspace/github/POD-XIAO-Firmware/scripts/

flash:
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) $(OPENOCD_CMDS) -f $(OPENOCD_TARGET) -c init -c targets -c "reset halt" \
                 -c "flash write_image erase $(PROG).bin $(LOAD_ADDRESS) bin" -c "reset run" -c shutdown

gdb:
	$(OPENOCD) -f $(OPENOCD_INTERFACE) -f $(OPENOCD_TARGET)

sgdb:
	arm-none-eabi-gdb $(PROG).elf