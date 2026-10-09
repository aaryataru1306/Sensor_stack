# Shell configuration for Windows
SHELL = cmd.exe

# Target executable name
TARGET = Sensor_Stack

# Toolchain
CC      = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump
SIZE    = arm-none-eabi-size

# STM32F411CEU6 - Cortex-M4 with FPU
CPU     = -mcpu=cortex-m4
FPU     = -mfpu=fpv4-sp-d16
FLOAT   = -mfloat-abi=hard
MCU     = $(CPU) -mthumb $(FPU) $(FLOAT)

# Target MCU
DEFS = -DSTM32F411xE

# Header include paths
INCLUDES = \
  -IApplication \
  -ICore/Inc \
  -ICore/CMSIS/Device/ST/STM32F4xx/Include \
  -ICore/CMSIS/Include \
  -IProtocols/I2C \
  -IProtocols/UART \
  -ISensors/QMC5883P/Inc

# Source files: QMC5883P + I2C + UART
C_SOURCES = \
  Application/main.c \
  Protocols/I2C/i2c.c \
  Protocols/UART/uart.c \
  Sensors/QMC5883P/Src/qmc5883p.c

# Core system sources
C_SOURCES += $(wildcard Core/Src/*.c)

# Startup assembly
ASM_SOURCES = Startup/startup_stm32f411xe.s

# Linker script
LDSCRIPT = Linker/stm32f411ceux_flash.ld

# Compiler flags
CFLAGS = $(MCU) $(DEFS) $(INCLUDES) \
         -O2 -Wall \
         -fdata-sections -ffunction-sections

# Linker flags
LDFLAGS = $(MCU) \
          -T$(LDSCRIPT) \
          -Wl,--gc-sections \
          -Wl,-Map=build/$(TARGET).map \
          --specs=nano.specs \
          -lc -lm -lnosys

# Build directory
BUILD_DIR = build

# Object files
OBJECTS = $(addprefix $(BUILD_DIR)/,$(C_SOURCES:.c=.o))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(ASM_SOURCES:.s=.o))

# Default build target
all: $(BUILD_DIR)/$(TARGET).elf \
     $(BUILD_DIR)/$(TARGET).hex \
     $(BUILD_DIR)/$(TARGET).bin
	@$(SIZE) $(BUILD_DIR)/$(TARGET).elf

# Compile C sources
$(BUILD_DIR)/%.o: %.c
	@if not exist "$(subst /,\,$(dir $@))" mkdir "$(subst /,\,$(dir $@))"
	@echo Compiling: $<
	$(CC) -c $(CFLAGS) $< -o $@

# Assemble startup file
$(BUILD_DIR)/%.o: %.s
	@if not exist "$(subst /,\,$(dir $@))" mkdir "$(subst /,\,$(dir $@))"
	@echo Assembling: $<
	$(CC) -c $(CFLAGS) $< -o $@

# Link firmware
$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS)
	@echo Linking: $@
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@

# Generate Intel HEX
$(BUILD_DIR)/$(TARGET).hex: $(BUILD_DIR)/$(TARGET).elf
	@echo Creating HEX file: $@
	$(OBJCOPY) -O ihex $< $@

# Generate binary
$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET).elf
	@echo Creating BIN file: $@
	$(OBJCOPY) -O binary -S $< $@

# Clean build artifacts
clean:
	@if exist "$(BUILD_DIR)" rmdir /s /q "$(BUILD_DIR)"
	@echo Cleaned build directory.

# Flash using ST-Link and OpenOCD
flash: all
	openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
	  -c "program $(BUILD_DIR)/$(TARGET).hex verify reset exit"

.PHONY: all clean flash