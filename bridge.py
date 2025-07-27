import serial
import time

class SiBr:
    def __init__(self, port="/dev/ttyUSB0", timeout=0.01):

        self.ser = serial.Serial(port=port,
                                 baudrate=9600,
                                 timeout=timeout, 
                                 bytesize=8,      # 8 data bits
                                 parity='N',      # No parity
                                 stopbits=1,      # 1 stop bit
                                 xonxoff=False,   # No software flow control
                                 rtscts=False,    # No hardware flow control
                                 dsrdtr=False     # No hardware flow control
        )
    
    def i2c_config(self, frequency_khz = 100):
        config_message = (b"\x00" + 
                          b"\x03" + 
                          b"\x01" + 
                          frequency_khz.to_bytes(2, 'big', signed=False) + 
                          b"\r\n")
    
        self.ser.write(config_message)
        return config_message
    
    def i2c_write(self, payload):
        if not isinstance(payload, bytes):
            payload = bytes(payload)
        
        full_message = (
            b"\x01" +                      # I2C channel
            (len(payload) + 1).to_bytes(1, 'little') +  # Length (write_cmd + payload)
            b"\x01" +                      # I2C_WRITE command
            payload +                      # Actual data (which will include address of course)
            b"\r\n"                        # Terminator
        )
        
        self.ser.write(full_message)
        
        return full_message
    
    
    def read(self, howmany=1024):
        print(self.ser.read(howmany))

    def close(self):
        self.ser.close()



