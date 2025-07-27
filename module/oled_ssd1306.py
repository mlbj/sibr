from ..bridge import SiBr
import time

class OLED_SSD1306:
    def __init__(self, bridge, width=128, height=64, address=0x3C, frequency_khz=100):
        """
        Args:
            bridge: Initialized SiBr instance
            width: Display width (default 128)
            height: Display height (default 64)
            address: I2C address (default 0x3C)
            frequency_khz: i2c clock frequency in kilohertz
        """
        if not isinstance(bridge, SiBr):
            raise TypeError("bridge must be an instance of SiBr")

        self.bridge = bridge
        self.width = width
        self.height = height
        self.address = bytes([address])

        # Start i2c
        self.bridge.i2c_config(frequency_khz)

        # Init display
        self._setup()

    def _setup(self):
        # Display off
        self.bridge.i2c_write(self.address + b"\x00\xAE")      

        # Set display clock div (reset value)
        self.bridge.i2c_write(self.address + b"\x00\xD5\x80") 
        
        # Set multiplex ratio
        if self.height == 64:
            self.bridge.i2c_write(self.address + b"\x00\xA8\x3F")
        elif self.height == 32:
            self.bridge.i2c_write(self.address + b"\x00\xA8\x1F") 

        # Set display offset (0)
        self.bridge.i2c_write(self.address + b"\x00\xD3\x00") 

        # Set start line (0)
        self.bridge.i2c_write(self.address + b"\x00\x40")     

        # Enable charge pump 
        self.bridge.i2c_write(self.address + b"\x00\x8D\x14") 

        # Horizontal addressing mode
        self.bridge.i2c_write(self.address + b"\x00\x20\x00")  

        # Segment remap (flips horizontally)
        self.bridge.i2c_write(self.address + b"\x00\xA1")    

        # COM output scan (flips vertically)
        self.bridge.i2c_write(self.address + b"\x00\xC8")      
        
        # COM pins config 
        if self.height == 64:
            self.bridge.i2c_write(self.address + b"\x00\xDA\x12")   
        elif self.height == 32:
            self.bridge.i2c_write(self.address + b"\x00\xDA\x02")  

        # Contrast to max
        self.bridge.i2c_write(self.address + b"\x00\x81\xFF")  

        # Pre-charge period
        self.bridge.i2c_write(self.address + b"\x00\xD9\xF1")  

        # VCOMH deselect level
        self.bridge.i2c_write(self.address + b"\x00\xDB\x40") 

        # Display follows RAM
        self.bridge.i2c_write(self.address + b"\x00\xA4")    

        # Normal display (Not inverted)
        self.bridge.i2c_write(self.address + b"\x00\xA6")  

        # Display on
        self.bridge.i2c_write(self.address + b"\x00\xAF")  

    def clear(self):
        """ Clear display """

        # Set column range
        self.bridge.i2c_write(self.address + b"\x00\x21" + bytes([0, self.width-1]))
        
        # Set page range
        self.bridge.i2c_write(self.address + b"\x00\x22" + bytes([0, (self.height//8)-1]))

        # Fill with zeros
        for _ in range(self.width*self.height//(32*8)):
            self.bridge.i2c_write(self.address + bytes([0x40]) + b'\x00'*32)
    
    def checkerboard(self):
        """ Draw a checkerboard """
        
        # Set column range
        self.bridge.i2c_write(self.address + b"\x00\x21" + bytes([0, self.width-1]))

        # Set page range
        self.bridge.i2c_write(self.address + b"\x00\x22" + bytes([0, (self.height//8)-1]))
        
        pattern = bytes([0x55, 0x00])
 
        # Fill with zeros
        for _ in range(self.width*self.height//(32*8*len(pattern))):
            self.bridge.i2c_write(self.address + bytes([0x40]) + pattern*32)


