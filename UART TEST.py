from machine import Pin, UART
import time
from tmc2209 import TMC2209

# Shared UART Pins (All 4 drivers connect to these same pins)
UART_TX_PIN = 0 # 1k resistor
UART_RX_PIN = 1 # After the 1k resistor, this pin connects to the RX pin of all 4 drivers in parallel
UART_BAUDRATE = 115200

STEPS_PER_TEST_SWEEP = 2000  
SPEED_DELAY_US = 800        

# Ensure each address matches the physical MS1/MS2 pin wiring
MOTOR_CONFIGS = [
    {
        "name": "Base",
        "address": 0, #MS1=0, MS2=0
        "step_pin": 16,
        "dir_pin": 17,
        "run_current_mA": 600,
        "hold_current_mA": 300,
        "microsteps": 16
    },
    {
        "name": "Shoulder",
        "address": 1, #MS1=1, MS2=0
        "step_pin": 14,
        "dir_pin": 15,
        "run_current_mA": 800, 
        "hold_current_mA": 400,
        "microsteps": 16
    },
    
    {
        "name": "Elbow",
        "address": 2, #MS1=0, MS2=1
        "step_pin": 12,
        "dir_pin": 13,
        "run_current_mA": 600,
        "hold_current_mA": 300,
        "microsteps": 16
    },
    {
        "name": "Wrist",
        "address": 3, #MS1=1, MS2=1
        "step_pin": 10,
        "dir_pin": 11,
        "run_current_mA": 400, 
        "hold_current_mA": 200,
        "microsteps": 16
    }
]

# Initialize the shared UART bus
uart = UART(0, baudrate=UART_BAUDRATE, tx=Pin(UART_TX_PIN), rx=Pin(UART_RX_PIN))

motors = []

for config in MOTOR_CONFIGS:
    
    dir_p = Pin(config['dir_pin'], Pin.OUT)
    step_p = Pin(config['step_pin'], Pin.OUT)
    
    tmc = TMC2209(uart, address=config['address'])
    
    # Clear old settings and apply new ones
    tmc.clear_gconf()
    tmc.set_motor_current(run_current_ma=config['run_current_mA'], hold_current_ma=config['hold_current_mA'])
    tmc.set_microstepping(config['microsteps'])
    tmc.set_stealthchop_enable(True)
    
    tmc.enable()
    
    # Store everything in a dictionary for easy access later
    motors.append({
        "name": config['name'],
        "tmc": tmc,
        "step_pin": step_p,
        "dir_pin": dir_p
    })

# Moves specific motor
def move_motor(motor_dict, steps, direction, delay_us):

    motor_dict["dir_pin"].value(direction)
    step_pin = motor_dict["step_pin"]
    
    for _ in range(steps):
        step_pin.high()
        time.sleep_us(delay_us)
        step_pin.low()
        time.sleep_us(delay_us)

try:
    
    while True:
        # Loop through each motor one by one
        for motor in motors:
            print(f"--> Moving {motor['name']}")
            
            # Move Forward
            move_motor(motor, STEPS_PER_TEST_SWEEP, 1, SPEED_DELAY_US)
            time.sleep(0.5)
            
            # Move Backward
            move_motor(motor, STEPS_PER_TEST_SWEEP, 0, SPEED_DELAY_US)
            time.sleep(1) # Pause before moving the next motor

except KeyboardInterrupt:
    print("\nTest interrupted by user.")

# Always disable all drivers on exit to prevent overheating
finally:
    for motor in motors:
        motor["tmc"].disable()