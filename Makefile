PROJECT_NAME     := tx_implant
TARGETS          := nrf52832_xxaa
OUTPUT_DIRECTORY := _build

SDK_ROOT := ../nRF5_SDK_15.2.0_9412b96
PROJ_DIR := .

$(OUTPUT_DIRECTORY)/nrf52832_xxaa.out: \
  LINKER_SCRIPT  := tx_implant_nrf52.ld

# Source files common to all targets
SRC_FILES += \
$(PROJ_DIR)/src/main.c \
$(SDK_ROOT)/modules/nrfx/mdk/system_nrf52.c \
$(PROJ_DIR)/src/flashwriting.c \
$(SDK_ROOT)/integration/nrfx/legacy/nrf_drv_spi.c \
$(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_gpiote.c \
$(SDK_ROOT)/components/proprietary_rf/esb/nrf_esb.c \
$(SDK_ROOT)/components/libraries/util/app_error.c \
$(SDK_ROOT)/components/libraries/util/app_error_weak.c \
$(SDK_ROOT)/components/libraries/util/app_util_platform.c \
$(SDK_ROOT)/components/libraries/util/nrf_assert.c \
$(PROJ_DIR)/external/RTT/SEGGER_RTT.c \
$(PROJ_DIR)/external/RTT/SEGGER_RTT_printf.c \
$(PROJ_DIR)/external/Syscalls/RTT_Syscalls_GCC.c \
$(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_timer.c \

# Include folders common to all targets
INC_FOLDERS += \
 $(SDK_ROOT)/components/toolchain/gcc \
 $(SDK_ROOT)/components/toolchain \
 $(PROJ_DIR) \
 $(PROJ_DIR)/resources \
 $(PROJ_DIR)/src \
 $(PROJ_DIR)/external/incbin-master \
 $(PROJ_DIR)/external/RTT \
 $(PROJ_DIR)/external/Syscalls \
 $(PROJ_DIR)/config \
 $(SDK_ROOT)/components \
 $(SDK_ROOT)/components/drivers_nrf/nrf_soc_nosd \
 $(SDK_ROOT)/components/libraries/uart \
 $(SDK_ROOT)/components/libraries/util \
 $(SDK_ROOT)/components/proprietary_rf/esb \
 $(SDK_ROOT)/components/toolchain/CMSIS/Include \
 $(SDK_ROOT)/components/libraries/timer \
 $(SDK_ROOT)/integration/nrfx/legacy \
 $(SDK_ROOT)/modules/nrfx/mdk \
 $(SDK_ROOT)/modules/nrfx \
  $(SDK_ROOT)/modules/nrfx/templates \
 $(SDK_ROOT)/modules/nrfx/templates/nRF52832 \
 $(SDK_ROOT)/modules/nrfx/drivers/include \
 $(SDK_ROOT)/modules/nrfx/hal \
 $(SDK_ROOT)/components/libraries/delay \
 $(SDK_ROOT)/components/libraries/log \
 $(SDK_ROOT)/components/libraries/log/src \
 $(SDK_ROOT)/components/libraries/memobj \
 $(SDK_ROOT)/components/libraries/balloc \
 $(SDK_ROOT)/components/libraries/experimental_section_vars \
 $(SDK_ROOT)/components/libraries/strerror \
 $(SDK_ROOT)/components/boards \
 $(SDK_ROOT)/components/libraries/bootloader/dfu \
 $(SDK_ROOT)/components/softdevice/s132/headers/nrf52 \
 $(SDK_ROOT)/components/softdevice/s132/headers \
  
 
# Libraries common to all targets
LIB_FILES += \

# Optimization flags
OPT = -O3 -g3
# Uncomment the line below to enable link time optimization
#OPT += -flto

# C flags common to all targets
CFLAGS += $(OPT)
CFLAGS += -DBOARD_CUSTOM
CFLAGS += -DINCLUDE_FPGA_IMAGE
CFLAGS += -DSDK15_2
#CFLAGS += -DUSE_BOOTLOADER
CFLAGS += -DBSP_DEFINES_ONLY
CFLAGS += -DCONFIG_GPIO_AS_PINRESET
CFLAGS += -DFLOAT_ABI_HARD
CFLAGS += -DNRF52
CFLAGS += -DNRF52832_XXAA
CFLAGS += -DNRF52_PAN_74
CFLAGS += -mcpu=cortex-m4
CFLAGS += -mthumb -mabi=aapcs
CFLAGS += -Wall -Werror
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# keep every function in a separate section, this allows linker to discard unused ones
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin -fshort-enums

# C++ flags common to all targets
CXXFLAGS += $(OPT)

# Assembler flags common to all targets
ASMFLAGS += -g3
ASMFLAGS += -mcpu=cortex-m4
ASMFLAGS += -mthumb -mabi=aapcs
ASMFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
ASMFLAGS += -DBOARD_CUSTOM
ASMFLAGS += -DBSP_DEFINES_ONLY
ASMFLAGS += -DCONFIG_GPIO_AS_PINRESET
ASMFLAGS += -DFLOAT_ABI_HARD
ASMFLAGS += -DNRF52
ASMFLAGS += -DNRF52832_XXAA
ASMFLAGS += -DNRF52_PAN_74

# Linker flags
LDFLAGS += $(OPT)
LDFLAGS += -mthumb -mabi=aapcs -L$(SDK_ROOT)/modules/nrfx/mdk -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m4
LDFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# let linker dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs

nrf52832_xxaa: CFLAGS += -D__HEAP_SIZE=8192
nrf52832_xxaa: CFLAGS += -D__STACK_SIZE=8192
nrf52832_xxaa: ASMFLAGS += -D__HEAP_SIZE=8192
nrf52832_xxaa: ASMFLAGS += -D__STACK_SIZE=8192

# Add standard libraries at the very end of the linker input, after all objects
# that may need symbols provided by these libraries.
LIB_FILES += -lc -lnosys -lm


.PHONY: default help

# Default target - first one defined
default: nrf52832_xxaa

# Print all targets that can be built
help:
	@echo following targets are available:
	@echo		nrf52832_xxaa
	@echo		sdk_config - starting external tool for editing sdk_config.h
	@echo		flash      - flashing binary

TEMPLATE_PATH := $(SDK_ROOT)/components/toolchain/gcc


include $(TEMPLATE_PATH)/Makefile.common

$(foreach target, $(TARGETS), $(call define_target, $(target)))

.PHONY: flash erase

# Flash the program
flash: default
	@echo Flashing: $(OUTPUT_DIRECTORY)/nrf52832_xxaa.hex
	nrfjprog -f nrf52 --program $(OUTPUT_DIRECTORY)/nrf52832_xxaa.hex --sectorerase
	nrfjprog -f nrf52 --reset

erase:
	nrfjprog -f nrf52 --eraseall

SDK_CONFIG_FILE := config/sdk_config.h
CMSIS_CONFIG_TOOL := $(SDK_ROOT)/external_tools/cmsisconfig/CMSIS_Configuration_Wizard.jar
sdk_config:
	java -jar $(CMSIS_CONFIG_TOOL) $(SDK_CONFIG_FILE)
