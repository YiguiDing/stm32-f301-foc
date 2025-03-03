# ------------------------------------------------
# Generic Makefile (based on gcc)
#
# ChangeLog :
#   2025-02-13 - 修改以实现对stm32f301c8t6标准库的编译
# 	2024-08-28 - 通过include关键字实现模块的拆分
# 					+ include Library/Makefile
# 					+ include Driver/Makefile
# 					+ include Core/Makefile
# 	2024-07-22 - 实现c/c++混合编译、调用openocd实现写入/擦除
#	2017-02-10 - Several enhancements + project update mode
#   2015-07-22 - first version
# ------------------------------------------------
######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -Og
#######################################
# paths
#######################################
BUILD_DIR = build
OUTPUT_DIR = output
######################################
# target
######################################
TARGET = main
######################################
# source
######################################
include library/FreeRTOS.mk
include library/StdPeriph.mk
include src/main.mk
# ASM sources
ASM_SOURCES +=  \
# C sources
C_SOURCES +=  \
# CPP sources
CPP_SOURCES +=  \
# AS includes
AS_INCLUDES += \
# C includes
C_CPP_INCLUDES +=  \
#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
XX = $(GCC_PATH)/$(PREFIX)g++
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
XX = $(PREFIX)g++
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
#######################################
# C_FLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m4

FLASH_START = 0x08000000
# fpu
# NONE for Cortex-M0/M0+/M3
# 对于STM32F301系列，FPU的类型是fpv4-sp-d16，表示单精度浮点运算单元。
FPU = -mfpu=fpv4-sp-d16

# float-abi
# FLOAT-ABI选项用于指定浮点运算的ABI（应用二进制接口）
# soft：软件浮点运算（不使用FPU）。
# softfp：使用FPU进行浮点运算，但函数调用时使用软件浮点ABI。
# hard：使用FPU进行浮点运算，并且函数调用时使用硬件浮点ABI。
FLOAT-ABI = -mfloat-abi=hard

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
	-D STM32F30X \
	-D STM32F301x8 \
	-D USE_STDPERIPH_DRIVER \
	-D ARM_MATH_CM4 \
	-D __BSD_VISIBLE=1 \
	-D __FPU_PRESENT=1

# compile gcc flags
ASM_FLAGS += $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections
C_FLAGS += $(MCU) $(C_DEFS) $(C_CPP_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections
CPP_FLAGS += $(MCU) $(C_DEFS) $(C_CPP_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
C_FLAGS += -g -gdwarf-2
CPP_FLAGS += -g -gdwarf-2
endif

# Generate dependency information
C_FLAGS += -MMD -MP -MF"$(@:%.o=%.d)"
CPP_FLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

#######################################
# LDFLAGS
#######################################
# link script
LD_SCRIPT += \

# libraries
LIBS = -lc -lm -lnosys 
LIBDIR = 
LDFLAGS = $(MCU) -specs=nosys.specs -T$(LD_SCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections,--print-memory-usage -u _printf_float

#######################################
# build the application
#######################################
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(CPP_SOURCES:.cpp=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))
vpath %.c $(sort $(dir $(C_SOURCES)))
vpath %.cpp $(sort $(dir $(CPP_SOURCES)))
#######################################
# build  objects
#######################################
$(BUILD_DIR):
	mkdir $@
$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(ASM_FLAGS) $< -o $@
$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	$(CC) -c $(C_FLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@
$(BUILD_DIR)/%.o: %.cpp Makefile | $(BUILD_DIR) 
	$(XX) -c $(CPP_FLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.cpp=.lst)) $< -o $@

#######################################
# build: .elf .hex .bin
#######################################
$(OUTPUT_DIR):
	mkdir $@
$(OUTPUT_DIR)/$(TARGET).elf: $(OBJECTS) Makefile | $(OUTPUT_DIR)
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@
$(OUTPUT_DIR)/%.hex: $(OUTPUT_DIR)/%.elf | $(OUTPUT_DIR)
	$(HEX) $< $@
$(OUTPUT_DIR)/%.bin: $(OUTPUT_DIR)/%.elf | $(OUTPUT_DIR)
	$(BIN) $< $@
#######################################
# default action: build all
#######################################
all: $(OUTPUT_DIR)/$(TARGET).elf $(OUTPUT_DIR)/$(TARGET).hex $(OUTPUT_DIR)/$(TARGET).bin
#######################################
# clean up
#######################################
clean:
	-rm -fR $(BUILD_DIR)
	-rm -fR $(OUTPUT_DIR)
#######################################
# write
#######################################
write: $(OUTPUT_DIR)/$(TARGET).bin
	openocd \
		-f interface/stlink.cfg \
		-f target/stm32f3x.cfg \
		-c "init; reset halt; wait_halt; flash write_image erase $(OUTPUT_DIR)/$(TARGET).bin ${FLASH_START}; reset; shutdown;" 
		@echo "Write Completed."
#######################################
# erase
#######################################
erase:
	openocd \
		-f interface/stlink.cfg \
		-f target/stm32f3x.cfg \
		-c "init; reset halt; flash erase_sector 0 0 1; exit"
	@echo "Erase Completed."
#######################################
# reset
#######################################
reset:
	openocd \
		-f interface/stlink.cfg \
		-f target/stm32f3x.cfg \
		-c "init; reset; exit"
	@echo "Erase Completed."
#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)
# *** EOF ***