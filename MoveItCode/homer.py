import time

def move_motor(motor_dict, steps, direction, delay_us):
    """Moves a motor a set number of steps blindly."""
    motor_dict["dir_pin"].value(direction)
    step_pin = motor_dict["step_pin"]
    
    for _ in range(steps):
        step_pin.high()
        time.sleep_us(delay_us)
        step_pin.low()
        time.sleep_us(delay_us)

def drive_until_trigger(motor_dict, direction, physical_limit_pin, override_button):
    """
    Drives a motor until EITHER the real limit switch is hit, 
    OR the master override button is pressed.
    """
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
        
        # Check if the real limit switch is hit
        if physical_limit_pin.value() == 0:
            return steps_taken
            
        # Check if the fake master override button is hit
        if override_button.value() == 0:
            print("  [Master Override Button Pressed!]")
            # Anti-bounce: wait for you to let go so you don't instantly skip the next phase
            while override_button.value() == 0:
                time.sleep(0.01)
            return steps_taken

def home_single(motor_dict, override_button):
    """Performs the 3-phase hybrid center-homing, with button override."""
    c = motor_dict['config']
    delay = c['delay_us']
    
    print(f"\nHoming {c['name']}...")
    
    # Phase 1: Move to MIN
    print("  -> Seeking MIN limit...")
    drive_until_trigger(motor_dict, c['homing_dir'], motor_dict['limit_min_pin'], override_button)
    move_motor(motor_dict, 200, not c['homing_dir'], delay) # Back off
    time.sleep(0.2) # Wait for mechanical bounce to settle

    # Phase 2: Move to MAX and measure
    print("  -> Seeking MAX limit...")
    total_width = drive_until_trigger(motor_dict, not c['homing_dir'], motor_dict['limit_max_pin'], override_button)
    total_width += 200 
    
    move_motor(motor_dict, 200, c['homing_dir'], delay) # Back off
    time.sleep(0.2)

    # Phase 3: Move to Mechanical Center
    center_steps = (total_width // 2) - 200
    print(f"  -> Mechanical center found. Moving back {center_steps} steps.")
    move_motor(motor_dict, center_steps, c['homing_dir'], delay)
    
    # Phase 4: Apply True Zero Offset
    offset = c.get('home_offset', 0) 
    if offset != 0:
        print(f"  -> Applying true vertical offset of {offset} steps.")
        if offset > 0:
            move_motor(motor_dict, offset, not c['homing_dir'], delay)
        else:
            move_motor(motor_dict, abs(offset), c['homing_dir'], delay)

    # ZERO the internal position counter
    motor_dict["current_position"] = 0
    motor_dict["target_position"] = 0
    print(f"✅ {c['name']} Homed and locked at True Zero.")

def home_all(motors_list, override_button):
    """Homes a list of multiple motors sequentially."""
    print("\n--- INITIATING HYBRID HOMING SEQUENCE ---")
    for m in motors_list:
        home_single(m, override_button)
    print("✅ All specified motors homed!")