import time
import struct

class TMC2209:
    """
    A unified MicroPython library for the TMC2209 stepper driver.
    Optimized for single-wire UART sharing on the Raspberry Pi Pico.
    """
    
    # TMC2209 Register Addresses
    REG_GCONF      = 0x00
    REG_IFCNT      = 0x02
    REG_IHOLD_IRUN = 0x10
    REG_CHOPCONF   = 0x6C

    def __init__(self, uart, address=0):
        self.uart = uart
        self.address = address
        # Slow 0.02 for 19200 baudrate
        self.communication_pause = 0.02 

    # =================================================================
    # LOW-LEVEL UART COMMUNICATION
    # =================================================================

    def compute_crc8_atm(self, datagram):
        crc = 0
        for byte in datagram:
            for _ in range(8):
                if (crc >> 7) ^ (byte & 0x01):
                    crc = ((crc << 1) ^ 0x07) & 0xFF
                else:
                    crc = (crc << 1) & 0xFF
                byte = byte >> 1
        return crc

    def read_reg(self, reg):
        """ Reads a register by accumulating bytes until the full frame arrives. """
        # Clear out any old garbage in the buffer
        while self.uart.any():
            self.uart.read() 

        # Build and send request datagram
        req = bytearray([0x55, self.address, reg, 0])
        req[3] = self.compute_crc8_atm(req[:-1])
        self.uart.write(req)
        
        # We expect a 12-byte total transaction (4 byte echo + 8 byte reply)
        # 100ms is a very safe timeout for 19200 baud
        timeout = time.ticks_ms() + 100
        resp = bytearray()
        
        while time.ticks_diff(timeout, time.ticks_ms()) > 0:
            if self.uart.any():
                resp.extend(self.uart.read())
            if len(resp) >= 12:
                break # We got the full message!
                
        # Scan the received buffer for the reply signature: Sync(0x05), Master(0xFF), Reg
        for i in range(len(resp) - 7):
            if resp[i] == 0x05 and resp[i+1] == 0xFF and resp[i+2] == reg:
                # Slicing the exact 4 bytes of data and the 1 byte CRC
                data_bytes = resp[i+3 : i+7]
                received_crc = resp[i+7]
                
                # Verify CRC of the reply (computed over the first 7 bytes of the reply)
                if self.compute_crc8_atm(resp[i : i+7]) == received_crc:
                    return data_bytes
                    
        return None # Return None if we timed out or CRC failed

    def read_int(self, reg):
        """ Tries to read a register and unpack it into an integer. """
        for _ in range(5): # Try up to 5 times
            data = self.read_reg(reg)
            if data is not None and len(data) == 4:
                return struct.unpack(">i", data)[0]
            time.sleep(0.01) # Breathe for 10ms before retrying
            
        print(f"TMC2209 (Address {self.address}) Read Error. Is motor power ON?")
        return 0

    def write_reg(self, reg, val):
        """ Writes a 32-bit value to a register. """
        frame = bytearray([0x55, self.address, reg | 0x80, 0, 0, 0, 0, 0])
        frame[3] = 0xFF & (val >> 24)
        frame[4] = 0xFF & (val >> 16)
        frame[5] = 0xFF & (val >> 8)
        frame[6] = 0xFF & val
        frame[7] = self.compute_crc8_atm(frame[:-1])
        
        self.uart.write(frame)
        time.sleep(0.01) # Give the driver time to process the write

    # =================================================================
    # HIGH-LEVEL MOTOR SETTINGS
    # =================================================================

    def clear_gconf(self):
        """ Resets Global Configuration to defaults. """
        self.write_reg(self.REG_GCONF, 0x00000000)

    def enable(self):
        """ Software enable: sets TOFF in CHOPCONF to default (3). """
        chopconf = self.read_int(self.REG_CHOPCONF)
        chopconf = (chopconf & 0xFFFFFFF0) | 0x03
        self.write_reg(self.REG_CHOPCONF, chopconf)

    def disable(self):
        """ Software disable: sets TOFF in CHOPCONF to 0. """
        chopconf = self.read_int(self.REG_CHOPCONF)
        chopconf = chopconf & 0xFFFFFFF0 
        self.write_reg(self.REG_CHOPCONF, chopconf)

    def set_stealthchop_enable(self, enable):
        """ True = StealthChop (Silent), False = SpreadCycle (More Torque). """
        gconf = self.read_int(self.REG_GCONF)
        if enable:
            gconf = gconf & ~(1 << 2)  # Clear bit 2 to enable
        else:
            gconf = gconf | (1 << 2)   # Set bit 2 to disable
        self.write_reg(self.REG_GCONF, gconf)

    def set_microstepping(self, microsteps):
        """ Sets microstep resolution (1, 2, 4, 8, 16, 32, 64, 128, 256). """
        ms_map = {256: 0, 128: 1, 64: 2, 32: 3, 16: 4, 8: 5, 4: 6, 2: 7, 1: 8}
        if microsteps not in ms_map:
            print("Invalid microstep value. Defaulting to 16.")
            microsteps = 16
            
        mres_value = ms_map[microsteps]
        chopconf = self.read_int(self.REG_CHOPCONF)
        # Clear bits 24-27
        chopconf = chopconf & ~(0xF << 24)
        # Set new bits
        chopconf = chopconf | (mres_value << 24)
        self.write_reg(self.REG_CHOPCONF, chopconf)

    def set_motor_current(self, run_current_ma, hold_current_ma):
        """ 
        Approximates the current to a 0-31 scale. 
        Max safe current mapping assumes ~1500mA-2000mA max hardware config.
        """
        # Convert mA to 0-31 range. 
        # (Divisor of ~50 is a safe approximation for standard 0.11 Ohm sense resistors)
        irun = min(31, max(0, int(run_current_ma / 50)))
        ihold = min(31, max(0, int(hold_current_ma / 50)))
        
        # IHOLDDELAY default is 1 (bits 16-19)
        val = (1 << 16) | (irun << 8) | ihold
        self.write_reg(self.REG_IHOLD_IRUN, val)