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
    REG_TCOOLTHRS  = 0x14  # Added for StallGuard
    REG_SGTHRS     = 0x40  # Added for StallGuard
    REG_SG_RESULT  = 0x41  # Added for StallGuard
    REG_CHOPCONF   = 0x6C

    def __init__(self, uart, address=0):
        self.uart = uart
        self.address = address
        self.communication_pause = 0.02 

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
        while self.uart.any():
            self.uart.read() 

        req = bytearray([0x55, self.address, reg, 0])
        req[3] = self.compute_crc8_atm(req[:-1])
        self.uart.write(req)
        
        timeout = time.ticks_ms() + 100
        resp = bytearray()
        
        while time.ticks_diff(timeout, time.ticks_ms()) > 0:
            if self.uart.any():
                resp.extend(self.uart.read())
            if len(resp) >= 12:
                break 
                
        for i in range(len(resp) - 7):
            if resp[i] == 0x05 and resp[i+1] == 0xFF and resp[i+2] == reg:
                data_bytes = resp[i+3 : i+7]
                received_crc = resp[i+7]
                
                if self.compute_crc8_atm(resp[i : i+7]) == received_crc:
                    return data_bytes
                    
        return None 

    def read_int(self, reg):
        for _ in range(5): 
            data = self.read_reg(reg)
            if data is not None and len(data) == 4:
                return struct.unpack(">i", data)[0]
            time.sleep(0.01) 
            
        print(f"TMC2209 (Address {self.address}) Read Error")
        return 0

    def write_reg(self, reg, val):
        frame = bytearray([0x55, self.address, reg | 0x80, 0, 0, 0, 0, 0])
        frame[3] = 0xFF & (val >> 24)
        frame[4] = 0xFF & (val >> 16)
        frame[5] = 0xFF & (val >> 8)
        frame[6] = 0xFF & val
        frame[7] = self.compute_crc8_atm(frame[:-1])
        
        self.uart.write(frame)
        time.sleep(0.01) 

    def clear_gconf(self):
        self.write_reg(self.REG_GCONF, 0x00000000)

    def enable(self):
        chopconf = self.read_int(self.REG_CHOPCONF)
        chopconf = (chopconf & 0xFFFFFFF0) | 0x03
        self.write_reg(self.REG_CHOPCONF, chopconf)

    def disable(self):
        chopconf = self.read_int(self.REG_CHOPCONF)
        chopconf = chopconf & 0xFFFFFFF0 
        self.write_reg(self.REG_CHOPCONF, chopconf)

    def set_stealthchop_enable(self, enable):
        gconf = self.read_int(self.REG_GCONF)
        if enable:
            gconf = gconf & ~(1 << 2)  
        else:
            gconf = gconf | (1 << 2)   
        self.write_reg(self.REG_GCONF, gconf)

    def set_microstepping(self, microsteps):
        ms_map = {256: 0, 128: 1, 64: 2, 32: 3, 16: 4, 8: 5, 4: 6, 2: 7, 1: 8}
        if microsteps not in ms_map:
            microsteps = 16
        mres_value = ms_map[microsteps]
        chopconf = self.read_int(self.REG_CHOPCONF)
        chopconf = chopconf & ~(0xF << 24)
        chopconf = chopconf | (mres_value << 24)
        self.write_reg(self.REG_CHOPCONF, chopconf)

    def set_motor_current(self, run_current_ma, hold_current_ma):
        irun = min(31, max(0, int(run_current_ma / 50)))
        ihold = min(31, max(0, int(hold_current_ma / 50)))
        val = (1 << 16) | (irun << 8) | ihold
        self.write_reg(self.REG_IHOLD_IRUN, val)

    def set_tcoolthrs(self, threshold):
        """ Lower limit velocity for StallGuard to be active (0xFFFFF = always active) """
        self.write_reg(self.REG_TCOOLTHRS, threshold)

    def set_sgthrs(self, sensitivity):
        """ StallGuard sensitivity (0 to 255). Higher = more sensitive to crashing. """
        self.write_reg(self.REG_SGTHRS, sensitivity & 0xFF)

    def get_sg_result(self):
        """ Live StallGuard reading. Drops to 0 when stalled/crashed. """
        return self.read_int(self.REG_SG_RESULT)