############################################################
# Project
############################################################
PROJECT = daq


############################################################
# Compiler Options
############################################################
OPTIMIZATION = s
STANDARD     = c11
ARCH         = -DSTM32F4 -march=armv7e-m -mthumb -mcpu=cortex-m4
FP           = -mfloat-abi=hard -mfpu=fpv4-sp-d16


############################################################
# Linker Options
############################################################
LDSCRIPT     = stm32f4-discovery.ld


############################################################
# Directories
############################################################
BUILD_DIR     = ./build
OPENCM3_DIR   = ./libopencm3
INCLUDE_DIR   = ./include ./libopencm3/include
LIBRARIES_DIR = ./lib 
SOURCE_DIR    = ./src
LDSCRIPT_DIR  = ./ld


############################################################
# Tools
############################################################
CC = arm-none-eabi-gcc
AR = arm-none-eabi-ar
LD = arm-none-eabi-gcc
GDB = arm-none-eabi-gdb
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump
SIZE = arm-none-eabi-size
REMOVE  = rm -rfv
STFLASH = st-flash


############################################################
# Source Files
############################################################
C_FILES = $(shell find $(SOURCE_DIR) -type f -name '*.c')
S_FILES = $(shell find $(SOURCE_DIR) -type f -name '*.S')
O_FILES = $(C_FILES:.c=.o) $(S_FILES:.S=.o)


############################################################
# Tool Flags
############################################################
CC_FLAGS  = -g -Os
CC_FLAGS += -std=$(STANDARD) $(ARCH) $(FP)
CC_FLAGS += -ffunction-sections -fdata-sections -Wall -Wstrict-prototypes
CC_FLAGS += $(patsubst %,-I%,$(INCLUDE_DIR)) -I.

LD_FLAGS  = $(ARCH) $(FP)
LD_FLAGS += -L$(OPENCM3_DIR)/lib -L$(LDSCRIPT_DIR) -T$(LDSCRIPT) -lopencm3_stm32f4
LD_FLAGS += --static -nostartfiles
LD_FLAGS += -Wl,--start-group -lc -lgcc -lnosys -Wl,--end-group


############################################################
# Targets: Actions
############################################################
.SUFFIXES: .c .eep .h .hex .o .elf .s .S
.PHONY: all bin elf flash clean

all: elf

elf: $(BUILD_DIR)/$(PROJECT).elf

bin: $(BUILD_DIR)/$(PROJECT).bin

flash: $(BUILD_DIR)/$(PROJECT).bin
	$(STFLASH) write $< 0x08000000

erase:
	$(STFLASH) erase

clean:
	$(REMOVE) src/*.o src/**/*.o build/*.elf build/*.hex build/*.bin


############################################################
# Targets: Output
############################################################
$(BUILD_DIR)/$(PROJECT).elf: libopencm3 $(O_FILES)
	$(LD) -o $@ $(O_FILES) $(LD_FLAGS)

$(BUILD_DIR)/$(PROJECT).bin: $(BUILD_DIR)/$(PROJECT).elf
	$(OBJCOPY) -Obinary $< $@

%.o: %.c
	$(CC) $(CC_FLAGS) -c $< -o $@

%.o: %.S
	$(CC) $(AS_FLAGS) -c $< -o $@

libopencm3:
	git submodule update
	make -C libopencm3
