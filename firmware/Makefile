# Target name
TARGET = main

# MCU settings
MCU = atmega328p
F_CPU = 16000000

# Toolchain
CC = avr-gcc
OBJCOPY = avr-objcopy
CFLAGS = -mmcu=$(MCU) -DF_CPU=$(F_CPU)UL -Os -Wall -Wextra -I.

# Source files
SRC = main.c 

.PHONY: all flash clean

all: $(TARGET).hex

$(TARGET).elf: $(SRC)
	$(CC) $(CFLAGS) -o $@ $(SRC)

%.hex: %.elf
	$(OBJCOPY) -O ihex $< $@

flash: $(TARGET).hex
	avrdude -p $(MCU) -c arduino -P /dev/ttyUSB0 -b 57600 -U flash:w:$<:i

clean:
	rm -f *.elf *.hex *.o
