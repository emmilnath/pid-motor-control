# Makefile for PID Motor Control firmware
# Supports two build targets:
#   make sim        — x86 simulation build (no hardware needed)
#   make TARGET=arm — ARM cross-compilation for STM32F411

TARGET ?= sim
BUILD_DIR = build/$(TARGET)

# Sources
SRCS = Src/main.c Src/pid.c Src/motor.c Src/encoder.c Src/serial_protocol.c

ifeq ($(TARGET),sim)
  CC = gcc
  CFLAGS = -Wall -Wextra -O2 -g -DSIMULATION_MODE -IInc
  LDFLAGS = -lm
  BIN = $(BUILD_DIR)/pid_sim
else
  CC = arm-none-eabi-gcc
  CFLAGS = -Wall -Wextra -Os -mcpu=cortex-m4 -mthumb -mfloat-abi=hard \
           -mfpu=fpv4-sp-d16 -DSTM32F411xE -IInc \
           -I../../Drivers/CMSIS/Include \
           -I../../Drivers/STM32F4xx_HAL_Driver/Inc
  LDFLAGS = -T STM32F411RETx_FLASH.ld -lm -lnosys --specs=nano.specs
  BIN = $(BUILD_DIR)/pid_firmware.elf
endif

OBJS = $(SRCS:%.c=$(BUILD_DIR)/%.o)

.PHONY: all sim arm clean

all: $(BIN)

sim:
	$(MAKE) TARGET=sim

arm:
	$(MAKE) TARGET=arm

$(BIN): $(OBJS)
	@mkdir -p $(dir $@)
	$(CC) $(OBJS) -o $@ $(LDFLAGS)

$(BUILD_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -rf build/
