from machine import Pin
import time

enable_pin = Pin(27, Pin.OUT)

def enable_motor():
    enable_pin.value(0)
def disable_motor();
    enable_pin.value(1)
    
# For safety
disable_motor()

# Motor class
class Motor:
    def __init__(self, name, dir_pin, step_pin):
        self.name = name
        self.dir = Pin(dir_pin, Pin.OUT)
        self.step = Pin(step_pin, Pin.OUT)

    def set_direction(self, forward=True):
        self.dir.value(1 if forward else 0)

    def pulse(self):
        """Sends a single microsecond pulse to take one step."""
        self.step.value(1)
        time.sleep_us(2)
        self.step.value(0)
