############################################################
# Project
############################################################
PROJECT = daq
DEVICE  = STM32F407xx


############################################################
# Compiler Options
############################################################
OPTIMIZATION = 0
STANDARD     = c11
ARCH         = -march=armv7e-m -mthumb -mcpu=cortex-m4
FP           = -mfloat-abi=hard -mfpu=fpv4-sp-d16


############################################################
# Linker Options
############################################################
LDSCRIPT     = stm32f407vg_flash.ld


############################################################
# Directories
############################################################
INCLUDE_DIR   = ./include ./lib/cmsis/include ./lib/hal/include ./lib/usb/include
SOURCE_DIR    = ./src ./lib/cmsis/src ./lib/hal/src ./lib/usb/src
LDSCRIPT_DIR  = ./lib/ld


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
REMOVE  = rm -fv
STFLASH = st-flash


############################################################
# Source Files
############################################################
C_FILES = $(shell find $(SOURCE_DIR) -type f -name '*.c')
S_FILES = $(shell find $(SOURCE_DIR) -type f -name '*.s')
O_FILES = $(C_FILES:.c=.o) $(S_FILES:.s=.o)


############################################################
# Tool Flags
############################################################
CC_FLAGS  = -g -Os
CC_FLAGS += -D$(DEVICE)
CC_FLAGS += -std=$(STANDARD) $(ARCH) $(FP)
CC_FLAGS += -ffunction-sections -fdata-sections -Wall -Wstrict-prototypes
CC_FLAGS += $(patsubst %,-I%,$(INCLUDE_DIR)) -I.

LD_FLAGS  = $(ARCH) $(FP)
LD_FLAGS += -L$(LDSCRIPT_DIR) -T$(LDSCRIPT)
LD_FLAGS += --static
LD_FLAGS += -Wl,--start-group -lc -lgcc -lnosys -Wl,--end-group


############################################################
# Targets: Actions
############################################################
.SUFFIXES: .bin .c .elf .eep .h .hex .o .elf .s
.PHONY: all bin dep elf flash clean clean-lib clean-all disasm disasm-all

all: elf

elf: $(PROJECT).elf

bin: $(PROJECT).bin

flash: $(PROJECT).bin
	$(STFLASH) write $< 0x08000000

erase:
	$(STFLASH) erase

clean:
	find src -name "*.o" -exec $(REMOVE) {} \;
	$(REMOVE) *.elf *.hex *.bin *.S

clean-lib:
	find lib -name "*.o" -exec $(REMOVE) {} \;

clean-all: clean clean-lib

disasm:
	$(OBJDUMP) -d $(PROJECT).elf > $(PROJECT).S

disasm-all:
	$(OBJDUMP) -D $(PROJECT).elf > $(PROJECT).S

############################################################
# Targets: Output
############################################################
$(PROJECT).elf: $(O_FILES)
	$(LD) -o $@ $(O_FILES) $(LD_FLAGS)

$(PROJECT).bin: $(PROJECT).elf
	$(OBJCOPY) -Obinary $< $@

%.o: %.c
	$(CC) $(CC_FLAGS) -c $< -o $@

%.o: %.s
	$(CC) $(AS_FLAGS) -c $< -o $@
