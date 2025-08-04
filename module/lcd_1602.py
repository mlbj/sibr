from ..bridge import SiBr
import time

class LCD_1602:
    def __init__(self, bridge, address=0x27, frequency_khz=100):
        """
        Args:
            bridge: Initialized SiBr instance
            address: I2C address of the LCD backpack (usually 0x27 or 0x3F)
            frequency_khz: I2C frequency in kilohertz
        """
        if not isinstance(bridge, SiBr):
            raise TypeError("bridge must be an instance of SiBr")

        self.bridge = bridge
        self.address = bytes([address])

        # Backligth ON
        self.backlight = 0x08  
        self._delay = 0.001

        # Setup I2C
        self.bridge.i2c_config(frequency_khz)

        # Init lcd
        self._setup()

    def _pulse_enable(self, data):
        # Enable high
        self.bridge.i2c_write(self.address + bytes([data | 0x04]))
        time.sleep(self._delay)
        
        # Enable low
        self.bridge.i2c_write(self.address + bytes([data & ~0x04]))
        time.sleep(self._delay)

    def _send_nibble(self, nibble, mode=0):
        data = (nibble & 0xF0) | mode | self.backlight
        self.bridge.i2c_write(self.address + bytes([data]))
        self._pulse_enable(data)

    def _send_byte(self, byte, mode=0):
        self._send_nibble(byte & 0xF0, mode)
        self._send_nibble((byte << 4) & 0xF0, mode)

    def _command(self, cmd):
        self._send_byte(cmd, mode=0x00)

    def _write_char(self, char):
        self._send_byte(ord(char), mode=0x01)

    def _setup(self):
        time.sleep(0.05)  # Wait after power

        # Init in 4-bit mode
        for _ in range(3):
            self._send_nibble(0x30)
            time.sleep(0.005)
        self._send_nibble(0x20)

        # Function set: 4-bit, 2-line, 5x8 dots
        self._command(0x28)

        # Display control: display ON, cursor OFF, blink OFF
        self._command(0x0C)

        # Clear display
        self.clear()

        # Entry mode set: increment, no shift
        self._command(0x06)

    def clear(self):
        self._command(0x01)
        time.sleep(0.002)

    def home(self):
        self._command(0x02)
        time.sleep(0.002)

    def set_cursor(self, col, row):
        row_offsets = [0x00, 0x40]
        self._command(0x80 | (col + row_offsets[row]))

    def write(self, text, col=0, row=0):
        self.set_cursor(col, row)
        for char in text:
            self._write_char(char)

