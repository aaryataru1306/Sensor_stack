TARGET = firmware_test

CC = arm-none-eabi-gcc
AS = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
SIZE = arm-none-eabi-size

CPU = -mcpu=cortex-m4 -mthumb
FPU = -mfpu=fpv4-sp-d16 -mfloat-abi=hard

CFLAGS = $(CPU) $(FPU) \
         -DSTM32F411xE \
         -Iinc \
         -I../Drivers/CMSIS/Core/Include \
		 -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include \
         -O0 \
         -g3 \
         -ffunction-sections \
         -fdata-sections \
         -Wall \
         -Wextra

LDFLAGS = $(CPU) $(FPU) \
          -Tlinker/STM32F411CEUX_FLASH.ld \
          --specs=nosys.specs \
          --specs=nano.specs \
          -Wl,--gc-sections \
          -Wl,-Map=$(TARGET).map

C_SOURCES = \
    src/main.c \
    src/system_stm32f4xx.c \
    src/spi_driver.c \
    src/icm20948.c

ASM_SOURCES = \
    startup/startup_stm32f411xe.s

C_OBJECTS = $(C_SOURCES:src/%.c=build/%.o)
ASM_OBJECTS = $(ASM_SOURCES:startup/%.s=build/%.o)

OBJECTS = $(C_OBJECTS) $(ASM_OBJECTS)

all: $(TARGET).elf $(TARGET).hex $(TARGET).bin

$(TARGET).elf: $(OBJECTS)
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SIZE) $@

build/%.o: src/%.c
	$(CC) $(CFLAGS) -c $< -o $@

build/%.o: startup/%.s
	$(AS) $(CPU) -c $< -o $@

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary $< $@

clean:
	powershell -Command "Remove-Item -Force -ErrorAction SilentlyContinue build\*.o, $(TARGET).elf, $(TARGET).hex, $(TARGET).bin, $(TARGET).map"

.PHONY: all clean