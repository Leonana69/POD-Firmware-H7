OPENOCD				?= openocd
OPENOCD_INTERFACE	?= interface/stlink.cfg
OPENOCD_TARGET    	?= target/stm32h7x.cfg
OPENOCD_CMDS		?=
LOAD_ADDRESS		?= 0x8020000
PROG				?= $(BUILD_DIR)/$(TARGET)

VPATH += Core/Src
C_SOURCES += 

all: $(PROG).bin

flash:
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) $(OPENOCD_CMDS) -f $(OPENOCD_TARGET) -c init -c targets -c "reset halt" \
    -c "flash write_image erase $(PROG).elf" -c "verify_image $(PROG).elf" -c "reset run" -c shutdown

flash-f:
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) $(OPENOCD_CMDS) -f $(OPENOCD_TARGET) -c init -c targets -c "reset halt" \
                 -c "flash write_image erase $(PROG).bin $(LOAD_ADDRESS) bin" -c "reset run" -c shutdown