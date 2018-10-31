PROJECT_NAME := tx_implant

SDK_ROOT := ../nRF5_SDK_15.2.0_9412b96
PROJ_DIR := .



export OUTPUT_FILENAME
#MAKEFILE_NAME := $(CURDIR)/$(word $(words $(MAKEFILE_LIST)),$(MAKEFILE_LIST))
MAKEFILE_NAME := $(MAKEFILE_LIST)
MAKEFILE_DIR := $(dir $(MAKEFILE_NAME) ) 


TEMPLATE_PATH = $(SDK_ROOT)/components/toolchain/gcc
ifeq ($(OS),Windows_NT)
include $(TEMPLATE_PATH)/Makefile.windows
else
include $(TEMPLATE_PATH)/Makefile.posix
endif

MK := mkdir
RM := rm -rf

#echo suspend
ifeq ("$(VERBOSE)","1")
NO_ECHO := 
else
NO_ECHO := @
endif

# Toolchain commands
CC              := '$(GNU_INSTALL_ROOT)$(GNU_PREFIX)-gcc'
AS              := '$(GNU_INSTALL_ROOT)$(GNU_PREFIX)-as'
AR              := '$(GNU_INSTALL_ROOT)$(GNU_PREFIX)-ar' -r
LD              := '$(GNU_INSTALL_ROOT)$(GNU_PREFIX)-ld'
NM              := '$(GNU_INSTALL_ROOT)$(GNU_PREFIX)-nm'
OBJDUMP         := '$(GNU_INSTALL_ROOT)$(GNU_PREFIX)-objdump'
OBJCOPY         := '$(GNU_INSTALL_ROOT)$(GNU_PREFIX)-objcopy'
SIZE            := '$(GNU_INSTALL_ROOT)$(GNU_PREFIX)-size'

#function for removing duplicates in a list
remduplicates = $(strip $(if $1,$(firstword $1) $(call remduplicates,$(filter-out $(firstword $1),$1))))

#source common to all targets
C_SOURCE_FILES += \
$(PROJ_DIR)/src/main.c \
$(SDK_ROOT)/modules/nrfx/mdk/system_nrf52.c \
$(PROJ_DIR)/src/flashwriting.c \
$(SDK_ROOT)/integration/nrfx/legacy/nrf_drv_spi.c \
$(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_spi.c \
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
$(SDK_ROOT)/modules/nrfx/drivers/src/prs/nrfx_prs.c \
$(SDK_ROOT)/components/libraries/log/src/nrf_log_frontend.c \
$(SDK_ROOT)/components/libraries/ringbuf/nrf_ringbuf.c \
$(SDK_ROOT)/components/libraries/atomic/nrf_atomic.c \
$(SDK_ROOT)/components/libraries/memobj/nrf_memobj.c \
$(SDK_ROOT)/components/libraries/balloc/nrf_balloc.c \
 


#$(SDK_ROOT)/components/libraries/bootloader_dfu/bootloader_util_arm.c \
#$(SDK_ROOT)/components/libraries/bootloader_dfu/dfu_app_handler.c \
#$(SDK_ROOT)/components/libraries/bootloader_dfu/ble_dfu.c \


#assembly files common to all targets
ASM_SOURCE_FILES  += \
$(SDK_ROOT)/modules/nrfx/mdk/gcc_startup_nrf52.S \



#includes common to all targets
INC_PATH += \
 -I$(SDK_ROOT)/components/toolchain/gcc \
 -I$(SDK_ROOT)/components/toolchain \
 -I$(PROJ_DIR) \
 -I$(PROJ_DIR)/resources \
 -I$(PROJ_DIR)/src \
 -I$(PROJ_DIR)/external/incbin-master \
 -I$(PROJ_DIR)/external/RTT \
 -I$(PROJ_DIR)/external/Syscalls \
 -I$(PROJ_DIR)/config \
 -I$(SDK_ROOT)/components \
 -I$(SDK_ROOT)/components/drivers_nrf/nrf_soc_nosd \
 -I$(SDK_ROOT)/components/libraries/uart \
 -I$(SDK_ROOT)/components/libraries/util \
 -I$(SDK_ROOT)/components/proprietary_rf/esb \
 -I$(SDK_ROOT)/components/toolchain/CMSIS/Include \
 -I$(SDK_ROOT)/components/libraries/timer \
 -I$(SDK_ROOT)/integration/nrfx/legacy \
 -I$(SDK_ROOT)/modules/nrfx/mdk \
 -I$(SDK_ROOT)/modules/nrfx \
  -I$(SDK_ROOT)/modules/nrfx/templates \
 -I$(SDK_ROOT)/modules/nrfx/templates/nRF52832 \
 -I$(SDK_ROOT)/modules/nrfx/drivers/include \
 -I$(SDK_ROOT)/modules/nrfx/drivers/src/prs \
 -I$(SDK_ROOT)/modules/nrfx/hal \
 -I$(SDK_ROOT)/components/libraries/delay \
 -I$(SDK_ROOT)/components/libraries/log \
 -I$(SDK_ROOT)/components/libraries/log/src \
 -I$(SDK_ROOT)/components/libraries/memobj \
 -I$(SDK_ROOT)/components/libraries/balloc \
  -I$(SDK_ROOT)/components/libraries/ringbuf \
 -I$(SDK_ROOT)/components/libraries/atomic \
 -I$(SDK_ROOT)/components/libraries/experimental_section_vars \
 -I$(SDK_ROOT)/components/libraries/strerror \
 -I$(SDK_ROOT)/components/boards \
 -I$(SDK_ROOT)/components/libraries/bootloader/dfu \
 -I$(SDK_ROOT)/components/softdevice/s132/headers/nrf52 \
 -I$(SDK_ROOT)/components/softdevice/s132/headers \
 -I$(SDK_ROOT)/external/fprintf \


OBJECT_DIRECTORY = _build
LISTING_DIRECTORY = $(OBJECT_DIRECTORY)
OUTPUT_BINARY_DIRECTORY = $(OBJECT_DIRECTORY)

# Sorting removes duplicates
BUILD_DIRECTORIES := $(sort $(OBJECT_DIRECTORY) $(OUTPUT_BINARY_DIRECTORY) $(LISTING_DIRECTORY) )

#flags common to all targets
CFLAGS  = -DNRF52_PAN_12
CFLAGS += -DNRF52_PAN_15
CFLAGS += -DNRF52_PAN_58
CFLAGS += -DNRF52_PAN_20
CFLAGS += -DNRF52_PAN_54
CFLAGS += -DNRF52_PAN_31
CFLAGS += -DNRF52_PAN_30
CFLAGS += -DNRF52_PAN_51
CFLAGS += -DNRF52_PAN_36
CFLAGS += -DNRF52_PAN_53
CFLAGS += -DCONFIG_GPIO_AS_PINRESET
CFLAGS += -DNRF52_PAN_64
CFLAGS += -DNRF52_PAN_55
CFLAGS += -DNRF52_PAN_62
CFLAGS += -DNRF52_PAN_63
CFLAGS += -DBOARD_CUSTOM
CFLAGS += -DINCLUDE_FPGA_IMAGE
#CFLAGS += -DUSE_BOOTLOADER
CFLAGS += -DSDK15_2
CFLAGS += -DNRF52
CFLAGS += -DBSP_DEFINES_ONLY
CFLAGS += -mcpu=cortex-m4
CFLAGS += -mthumb -mabi=aapcs --std=gnu99
CFLAGS += -Wall -Werror -O0 -g3
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# keep every function in separate section. This will allow linker to dump unused functions
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin --short-enums 
# keep every function in separate section. This will allow linker to dump unused functions
LDFLAGS += -Xlinker -Map=$(LISTING_DIRECTORY)/$(OUTPUT_FILENAME).map
LDFLAGS += -mthumb -mabi=aapcs -L $(SDK_ROOT)/modules/nrfx/mdk -T$(LINKER_SCRIPT)

LDFLAGS += -mcpu=cortex-m4
LDFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# let linker to dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs -lc -lnosys

# Assembler flags
ASMFLAGS += -x assembler-with-cpp
ASMFLAGS += -DNRF52_PAN_12
ASMFLAGS += -DNRF52_PAN_15
ASMFLAGS += -DNRF52_PAN_58
ASMFLAGS += -DNRF52_PAN_20
ASMFLAGS += -DNRF52_PAN_54
ASMFLAGS += -DNRF52_PAN_31
ASMFLAGS += -DNRF52_PAN_30
ASMFLAGS += -DNRF52_PAN_51
ASMFLAGS += -DNRF52_PAN_36
ASMFLAGS += -DNRF52_PAN_53
ASMFLAGS += -DCONFIG_GPIO_AS_PINRESET
ASMFLAGS += -DNRF52_PAN_64
ASMFLAGS += -DNRF52_PAN_55
ASMFLAGS += -DNRF52_PAN_62
ASMFLAGS += -DNRF52_PAN_63
ASMFLAGS += -DBOARD_CUSTOM
ASMFLAGS += -DNRF52
ASMFLAGS += -DBSP_DEFINES_ONLY

#default target - first one defined
default: clean nrf52832_xxaa

#building all targets
all: clean
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e cleanobj
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e nrf52832_xxaa

#target for printing all targets
help:
	@echo following targets are available:
	@echo 	nrf52832_xxaa

C_SOURCE_FILE_NAMES = $(notdir $(C_SOURCE_FILES))
C_PATHS = $(call remduplicates, $(dir $(C_SOURCE_FILES) ) )
C_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(C_SOURCE_FILE_NAMES:.c=.o) )

ASM_SOURCE_FILE_NAMES = $(notdir $(ASM_SOURCE_FILES))
ASM_PATHS = $(call remduplicates, $(dir $(ASM_SOURCE_FILES) ))
ASM_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(ASM_SOURCE_FILE_NAMES:.S=.o) )

vpath %.c $(C_PATHS)
vpath %.S $(ASM_PATHS)

OBJECTS = $(C_OBJECTS) $(ASM_OBJECTS)

nrf52832_xxaa: OUTPUT_FILENAME := nrf52832_xxaa
nrf52832_xxaa: LINKER_SCRIPT=tx_implant_nrf52.ld

nrf52832_xxaa: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -lm -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e finalize

## Create build directories
$(BUILD_DIRECTORIES):
	echo $(MAKEFILE_NAME)
	$(MK) $@

# Create objects from C SRC files
$(OBJECT_DIRECTORY)/%.o: %.c
	@echo Compiling file: $(notdir $<)
	$(NO_ECHO)$(CC) $(CFLAGS) $(INC_PATH) -c -o $@ $<

# Assemble files
$(OBJECT_DIRECTORY)/%.o: %.S
	@echo Assembly file: $(notdir $<)
	$(NO_ECHO)$(CC) $(ASMFLAGS) $(INC_PATH) -c -o $@ $<
# Link
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -lm -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
## Create binary .bin file from the .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	@echo Preparing: $(OUTPUT_FILENAME).bin
	$(NO_ECHO)$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin

## Create binary .hex file from the .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	@echo Preparing: $(OUTPUT_FILENAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex

finalize: genbin genhex echosize

genbin:
	@echo Preparing: $(OUTPUT_FILENAME).bin
	$(NO_ECHO)$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin

## Create binary .hex file from the .out file
genhex: 
	@echo Preparing: $(OUTPUT_FILENAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex
echosize:
	-@echo ''
	$(NO_ECHO)$(SIZE) $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	-@echo ''

clean:
	$(RM) $(BUILD_DIRECTORIES)

cleanobj:
	$(RM) $(BUILD_DIRECTORIES)/*.o
flash: nrf52832_xxaa
	@echo Flashing: $(OUTPUT_BINARY_DIRECTORY)/$<.hex
	nrfjprog --program $(OUTPUT_BINARY_DIRECTORY)/$<.hex -f nrf52  --chiperase
	nrfjprog --reset -f nrf52

## Flash softdevice