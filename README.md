# sibr

A minimal firmware for ATmega328P that bridges serial and I2C communication. It allows a host computer to send and receiving I2C messages over a serial interface, which is useful for probing or reverse engineering I2C peripherals. A companion Python library simplifies usage.

### Build and flash

Build the firmware with

```bash
cd sibr/firmware
make
```

This will compile the firmware into a `.hex` file, which you can flash to an ATmega328p by running

```bash
make flash
```

You may need to adapt the USB device path in the `Makefile` to match your own.

Note: Compilation requires `avr-gcc` and flashing requires `avrdude`. 

### Usage example 

```python
import sibr

# Create a bridge object
br = sibr.SiBr("/dev/ttyUSB0")

# Set I2C clock frequency to 100 khz
br.i2c_config(100)

# Write 0x01 to address 0x50
br.i2c_write(b"\x50\x01")  

# Read any response
br.read()

# Close bridge
br.close()
```

### Protocol (Serial <-> Firmware)

Each command sent/received over UART follows this frame format

```css
[channel][length][payload][\r\n]
```
where 

- `channel` can be one of the following
    - `0x00`: Configuration
    - `0x01`: I2C
    - `0xFF`: Echo
- `length` is the `payload` length
- `payload` is the actual message for that channel
- `\r\n` is a terminator

#### Commands

The first byte of every payload sent to the configuration channel is a command. The following table lists all available commands

| Command Name        | Code  | Parameters                               | Description                          |
|---------------------|-------|------------------------------------------|--------------------------------------|
| `CFG_I2C_SET_FREQ`  | 0x01  | 2-byte unsigned integer (big-endian)     | Set I2C clock frequency in kHz       |
| `CFG_I2C_SCAN`      | 0x02  | *(none)*                                 | Scan I2C bus for connected devices   |


The host can also send an `ANY_ACK` (`0x00`) command to test the communication.


### Modules 

Modules in `sibr.module` are scripted examples of peripherals that can be controlled through the bridge. They serve as ready-to-use drivers (e.g., LCD, OLED) and as templates for creating new modules to probe more complex devices under test.

#### Example: OLED SSD1306
```python
import sibr
from sibr.module.oled_ssd1306 import OLED_SSD1306
br = sibr.SiBr("/dev/ttyUSB0")
oled = OLED_SSD1306(br)
oled.clear()
oled.checkerboard()
```

This connects to the device, initializes the OLED, clears the screen, and draws a checkerboard pattern.
