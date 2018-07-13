TOPDIR := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))

OBJ = main.o systick.o

CROSS_PREFIX ?= arm-none-eabi-
CC=$(CROSS_PREFIX)gcc
OBJCOPY=$(CROSS_PREFIX)objcopy
SIZE=$(CROSS_PREFIX)size

CFLAGS = -Wall -Wextra -std=gnu99 -I../libopencm3/include -I ../common -DSTM32F1 -mcpu=cortex-m3 -mthumb -fno-common -ffreestanding -Og -g
LDFLAGS = -lopencm3_stm32f1 -L../libopencm3/lib -mthumb -mcpu=cortex-m3 -nostartfiles -Wl,--gc-sections -flto -T ../libopencm3/lib/stm32/f1/stm32f103x8.ld -g

all: firmware.bin

firmware.elf: $(OBJ)
	$(CC) -o firmware.elf $^ $(LDFLAGS)
	$(SIZE) -B firmware.elf

%.bin: %.elf
	$(OBJCOPY) -O binary $^ $@

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $^

clean:
	rm -f *.o firmware.elf firmware.bin

.PHONY: all flash clean debug
