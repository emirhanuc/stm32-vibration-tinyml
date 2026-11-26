# Standalone GCC build for PM_EDGE_NODE_V2 (STM32F407VG). Usage: make [OPT=-O2]
TARGET  := pm_edge_node
BUILD   := build
OPT     ?= -O2
CC      := arm-none-eabi-gcc
SIZE    := arm-none-eabi-size
OBJCOPY := arm-none-eabi-objcopy

MCU := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard

C_SRCS := $(wildcard Core/Src/*.c) \
          Drivers/LIS3DSH/lis3dsh_driver.c \
          $(wildcard Services/Sampling/*.c) \
          $(wildcard Services/Inference/*.c) \
          $(wildcard USB_DEVICE/App/*.c) $(wildcard USB_DEVICE/Target/*.c) \
          $(wildcard Middlewares/ST/STM32_USB_Device_Library/Core/Src/*.c) \
          $(wildcard Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/*.c) \
          $(filter-out %_template.c,$(wildcard Drivers/STM32F4xx_HAL_Driver/Src/*.c))
ASM_SRCS := Core/Startup/startup_stm32f407vgtx.s

INCS := -ICore/Inc -IDrivers/LIS3DSH -IServices/Sampling -IServices/Inference \
        -IServices/Inference/generated -IUSB_DEVICE/App -IUSB_DEVICE/Target \
        -IDrivers/STM32F4xx_HAL_Driver/Inc -IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy \
        -IDrivers/CMSIS/Device/ST/STM32F4xx/Include -IDrivers/CMSIS/Include \
        -IMiddlewares/ST/STM32_USB_Device_Library/Core/Inc \
        -IMiddlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc

CFLAGS  := $(MCU) -std=gnu11 $(OPT) -g3 -DUSE_HAL_DRIVER -DSTM32F407xx $(INCS) \
           -ffunction-sections -fdata-sections -Wall -MMD -MP
LDFLAGS := $(MCU) -TSTM32F407VGTX_FLASH.ld --specs=nano.specs --specs=nosys.specs \
           -Wl,-Map=$(BUILD)/$(TARGET).map -Wl,--gc-sections -Wl,--print-memory-usage \
           -Wl,--start-group -lc -lm -Wl,--end-group

OBJS := $(addprefix $(BUILD)/,$(C_SRCS:.c=.o) $(ASM_SRCS:.s=.o))

all: $(BUILD)/$(TARGET).elf $(BUILD)/$(TARGET).bin

$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/%.o: %.s
	@mkdir -p $(dir $@)
	$(CC) $(MCU) -x assembler-with-cpp -c $< -o $@

$(BUILD)/$(TARGET).elf: $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) -o $@
	$(SIZE) $@

$(BUILD)/$(TARGET).bin: $(BUILD)/$(TARGET).elf
	$(OBJCOPY) -O binary $< $@

flash: $(BUILD)/$(TARGET).bin
	st-flash --reset write $< 0x08000000

clean:
	rm -rf $(BUILD)

-include $(OBJS:.o=.d)
.PHONY: all clean flash
