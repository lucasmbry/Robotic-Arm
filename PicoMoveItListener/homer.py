import time

# Move motor number of steps
def move_motor(motor_dict, steps, direction, delay_us):

    motor_dict["dir_pin"].value(direction)
    step_pin = motor_dict["step_pin"]
    
    for _ in range(steps):
        step_pin.high()
        time.sleep_us(delay_us)
        step_pin.low()
        time.sleep_us(delay_us)

# Drives till either limit hit or override
def drive_until_trigger(motor_dict, direction, physical_limit_pin, override_button):

    motor_dict["dir_pin"].value(direction)
    step_pin = motor_dict["step_pin"]
    delay_us = motor_dict["config"]["delay_us"]
    steps_taken = 0
    
    while True:
        step_pin.high()
        time.sleep_us(delay_us)
        step_pin.low()
        time.sleep_us(delay_us)
        steps_taken += 1
        
        if physical_limit_pin.value() == 0:
            return steps_taken
            
        if override_button.value() == 0:
            print("Stopped")
            while override_button.value() == 0:
                time.sleep(0.01)
            return steps_taken

# Homing with override and offset
def home_single(motor_dict, override_button):

    c = motor_dict['config']
    delay = c['delay_us']
    
    print(f"\nHoming {c['name']}...")
    
    # Min
    print("  -> Seeking MIN limit...")
    drive_until_trigger(motor_dict, c['homing_dir'], motor_dict['limit_min_pin'], override_button)
    move_motor(motor_dict, 200, not c['homing_dir'], delay) # Back off
    time.sleep(0.2) # Wait for mechanical bounce to settle

    # Max
    print("  -> Seeking MAX limit...")
    total_width = drive_until_trigger(motor_dict, not c['homing_dir'], motor_dict['limit_max_pin'], override_button)
    total_width += 200 
    
    move_motor(motor_dict, 200, c['homing_dir'], delay) # Back off
    time.sleep(0.2)

    # Center
    center_steps = (total_width // 2) - 200
    print(f"  -> Mechanical center found. Moving back {center_steps} steps.")
    move_motor(motor_dict, center_steps, c['homing_dir'], delay)
    
    # Offset
    offset = c.get('home_offset', 0) 
    if offset != 0:
        print(f"  -> Applying true vertical offset of {offset} steps.")
        if offset > 0:
            move_motor(motor_dict, offset, not c['homing_dir'], delay)
        else:
            move_motor(motor_dict, abs(offset), c['homing_dir'], delay)

    # Zero
    motor_dict["current_position"] = 0
    motor_dict["target_position"] = 0
    print(f"✅ {c['name']} Homed and locked at True Zero.")

def home_all(motors_list, override_button):
    """Homes a list of multiple motors sequentially."""
    print("\n--- INITIATING HYBRID HOMING SEQUENCE ---")
    for m in motors_list:
        home_single(m, override_button)
    print("✅ All specified motors homed!")