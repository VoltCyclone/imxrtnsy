TOOLCHAIN = /Users/ramseymcgrath/.platformio/packages/toolchain-gccarmnoneeabi-teensy/bin
CC      = $(TOOLCHAIN)/arm-none-eabi-gcc
OBJCOPY = $(TOOLCHAIN)/arm-none-eabi-objcopy
SIZE    = $(TOOLCHAIN)/arm-none-eabi-size

TARGET  = firmware

MCU_FLAGS = -mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb

# Pass UART=1 on command line to enable debug UART output (e.g. make UART=1)
UART ?= 0
# Override UART baud rate (e.g. make UART_BAUD=921600)
UART_BAUD ?= 115200

DEFINES = -DARDUINO_TEENSY41 -D__IMXRT1062__ -DF_CPU=600000000 -DUART_ENABLED=$(UART) \
          -DUART_BAUD=$(UART_BAUD)

CFLAGS = $(MCU_FLAGS) $(DEFINES) \
         -Os -Wall -Wno-unused-variable \
         -ffunction-sections -fdata-sections \
         -Iinclude -Isrc

LDFLAGS = $(MCU_FLAGS) \
          -Tcore/imxrt1062_t41.ld \
          -Wl,--gc-sections \
          --specs=nano.specs --specs=nosys.specs

CORE_SRC = core/startup.c core/bootdata.c
SRC      = src/main.c src/uart.c src/usb_host.c src/usb_device.c src/desc_capture.c \
           src/kmbox.c src/humanize.c src/smooth.c src/ferrum.c src/makcu.c

OBJ = $(CORE_SRC:.c=.o) $(SRC:.c=.o)

all: $(TARGET).hex
	@$(SIZE) $(TARGET).elf

$(TARGET).elf: $(OBJ)
	$(CC) $(LDFLAGS) -o $@ $^ -lm

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@

# Hot-path sources get -O2 instead of -Os for better inlining/unrolling
HOT_SRC = src/usb_host.o src/usb_device.o src/kmbox.o src/smooth.o src/humanize.o
$(HOT_SRC): CFLAGS := $(subst -Os,-O2,$(CFLAGS))

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

flash: $(TARGET).hex
	teensy_loader_cli --mcu=TEENSY41 -w -v $(TARGET).hex

clean:
	rm -f $(OBJ) $(TARGET).elf $(TARGET).hex

.PHONY: all flash clean
