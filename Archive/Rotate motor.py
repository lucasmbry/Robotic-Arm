from machine import Pin
import time

# 1. Setup the Pico Pins
step_pin = Pin(15, Pin.OUT)
dir_pin = Pin(14, Pin.OUT)
enable_pin = Pin(13, Pin.OUT)

# 2. Enable the TMC2209
enable_pin.value(0) 

steps_per_revolution = 200  
microstepping = 8           
total_steps = steps_per_revolution * microstepping * 2 # Let's spin 2 full revolutions so we can see the top speed!

def move_motor_with_ramp(steps, direction, min_delay_us):
    """
    Moves the stepper motor using a Trapezoidal Acceleration Ramp.
    """
    dir_pin.value(direction)
    time.sleep_ms(1) # Let the direction pin settle
    
    start_delay = 1000 # Start slow (1 millisecond delay)
    ramp_steps = 400   # Take 400 steps to accelerate, and 400 to decelerate
    
    for i in range(steps):
        # --- CALCULATE THE RAMP ---
        if i < ramp_steps:
            # Accelerating: Interpolate from start_delay down to min_delay_us
            progress = i / ramp_steps
            current_delay = int(start_delay - (progress * (start_delay - min_delay_us)))
            
        elif i > (steps - ramp_steps):
            # Decelerating: Interpolate from min_delay_us back up to start_delay
            progress = (steps - i) / ramp_steps
            current_delay = int(start_delay - (progress * (start_delay - min_delay_us)))
            
        else:
            # Cruising at top speed
            current_delay = min_delay_us

        # --- EXECUTE THE STEP ---
        step_pin.value(1)
        time.sleep_us(2)  
        step_pin.value(0)
        
        # Wait based on our calculated ramp curve
        time.sleep_us(current_delay)

try:
    print("Starting Accelerated Motor Test...")
    
    while True:
        print("Ramping Forward to Top Speed...")
        # Now we can safely pass '60' because it will ramp up to it!
        move_motor_with_ramp(total_steps, direction=1, min_delay_us=50)
        
        time.sleep(1) 
        
        print("Ramping Backward to Top Speed...")
        move_motor_with_ramp(total_steps, direction=0, min_delay_us=50)
        
        time.sleep(1) 

except KeyboardInterrupt:
    print("Test stopped. Disabling motor.")
    enable_pin.value(1)