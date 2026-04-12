from machine import Pin
import time

enable_pin = Pin(27, Pin.OUT)

def enable_motor():
    enable_pin.value(0)
def disable_motor():
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

motor1 = Motor("Base", dir_pin=16, step_pin=17)
motor2 = Motor("Wrist", dir_pin=18, step_pin=19)
all_motors = [motor1, motor2]

def move_all(steps, direction, delay_us):
    for motor in all_motors:
        motor.set_direction(forward=direction)
        
    time.sleep_ms(1)
    
    for _ in range(steps):
        for motor in all_motors:
            motor.pulse()
            
        time.sleep_us(delay_us)
    
try:
    enable_motor()
    time.sleep(1)
    
    while True:
        
        move_all(steps=50000, direction=True, delay_us=100)
        
        time.sleep(1)
        
        move_all(steps=5000, direction=False, delay_us=100)
        
        time.sleep(1)
        
except KeyboardInterrupt:
    disable_motor()