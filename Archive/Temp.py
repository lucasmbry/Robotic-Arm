from machine import Pin, UART
import time 
import sys
import select
import math
import micropython
from Archive.tmc2209 import TMC2209

# --- SAFETY WINDOW ---
print("Booting... 3 second safety window active.")
time.sleep(3)
micropython.kbd_intr(-1)

UART_TX_PIN = 0 
UART_RX_PIN = 1 
UART_BAUDRATE = 19200

# Define master override button pin from advanced script
MANUAL_OVERRIDE_PIN = 15
manual_btn = Pin(MANUAL_OVERRIDE_PIN, Pin.IN, Pin.PULL_UP)

# Updated to match configuration parameters, microsteps, speeds, and accelerations exactly
MOTOR_CONFIGS = [
    {"name": "Base",     "address": 0, "step_pin": 17, "dir_pin": 16, "limit_min": 10, "limit_max": 11, "run_current_mA": 800,  "hold_current_mA": 300, "microsteps": 8, "homing_dir": 0, "gear_ratio": 15.0, "dir_invert": 1, "max_speed_sps": 2000, "max_accel_sps2": 8000},
    {"name": "Shoulder", "address": 1, "step_pin": 12, "dir_pin": 13, "limit_min": 4,  "limit_max": 5,  "run_current_mA": 1200, "hold_current_mA": 300, "microsteps": 8, "homing_dir": 0, "gear_ratio": 40.0, "dir_invert": 1, "max_speed_sps": 5000, "max_accel_sps2": 8000},
    {"name": "Elbow",    "address": 2, "step_pin": 19, "dir_pin": 18, "limit_min": 6,  "limit_max": 7,  "run_current_mA": 800,  "hold_current_mA": 300, "microsteps": 8, "homing_dir": 0, "gear_ratio": 20.0, "dir_invert": -1, "max_speed_sps": 3000, "max_accel_sps2": 8000},
    {"name": "Wrist",    "address": 3, "step_pin": 21, "dir_pin": 20, "limit_min": 8,  "limit_max": 8,  "run_current_mA": 800,  "hold_current_mA": 300, "microsteps": 8, "homing_dir": 0, "gear_ratio": 1.0,  "dir_invert": 1, "max_speed_sps": 5000, "max_accel_sps2": 8000},
]

poller = select.poll()
poller.register(sys.stdin, select.POLLIN)

uart = UART(0, baudrate=UART_BAUDRATE, tx=Pin(UART_TX_PIN), rx=Pin(UART_RX_PIN))
motors = []

print("Initializing Drivers and executing Diagnostic Readback...")
for config in MOTOR_CONFIGS:
    dir_p = Pin(config['dir_pin'], Pin.OUT)
    step_p = Pin(config['step_pin'], Pin.OUT)
    
    limit_min_p = Pin(config['limit_min'], Pin.IN, Pin.PULL_UP) 
    limit_max_p = Pin(config['limit_max'], Pin.IN, Pin.PULL_UP) 
    
    tmc = TMC2209(uart, address=config['address'])
    tmc.clear_gconf()
    
    # Apply the specific GCONF register modification from advanced configuration
    gconf = tmc.read_int(tmc.REG_GCONF)
    tmc.write_reg(tmc.REG_GCONF, gconf | (1 << 6) | (1 << 7))
    
    tmc.set_motor_current(run_current_ma=config['run_current_mA'], hold_current_ma=config['hold_current_mA'])
    tmc.set_microstepping(config['microsteps'])
    tmc.set_stealthchop_enable(True)
    tmc.enable()
    
    # Diagnostic readback confirmation sequence
    ifcnt = tmc.read_int(0x02)
    gconf_after = tmc.read_int(tmc.REG_GCONF)
    chopconf = tmc.read_int(0x6C)
    mres = (chopconf >> 24) & 0x0F
    mres_table = {0:256, 1:128, 2:64, 3:32, 4:16, 5:8, 6:4, 7:2, 8:1}
    actual_microsteps = mres_table.get(mres, "UNKNOWN")

    print(f"[DIAG] {config['name']:>8} addr={config['address']} "
          f"IFCNT={ifcnt} GCONF=0x{gconf_after:08X} "
          f"mstep_reg_select={bool(gconf_after & (1<<7))} "
          f"MRES_raw={mres} actual_microsteps={actual_microsteps}")
    
    motors.append({
        "name": config['name'],
        "tmc": tmc,
        "step_pin": step_p,
        "dir_pin": dir_p,
        "limit_min_pin": limit_min_p,
        "limit_max_pin": limit_max_p,
        "config": config,
        "max_speed": config['max_speed_sps'],
        "max_accel": config['max_accel_sps2']
    })

def move_motor(motor_dict, steps, direction, check_limits=True):
    """
    Moves the motor utilizing a linear trapezoidal acceleration/deceleration ramp
    based on max_speed_sps and max_accel_sps2 parameters.
    """
    c = motor_dict["config"]
    
    # Calculate electrical direction pin state based on config rules
    is_forward = (direction == 1)
    if c['dir_invert'] == -1:
        is_forward = not is_forward
    target_dir_val = 0 if c['homing_dir'] == 0 else 1
    if is_forward:
        target_dir_val = 1 if target_dir_val == 0 else 0
        
    motor_dict["dir_pin"].value(target_dir_val)
    step_pin = motor_dict["step_pin"]
    min_pin = motor_dict["limit_min_pin"]
    max_pin = motor_dict["limit_max_pin"]
    
    max_speed = motor_dict["max_speed"]
    max_accel = motor_dict["max_accel"]
    
    # Calculate ramping profiles
    min_speed = 40.0  # Starting steps per second
    current_speed = min_speed
    steps_taken = 0
    
    # Calculate how many steps it takes to reach full speed
    # s = (v^2 - u^2) / (2a)
    accel_steps = int((max_speed**2 - min_speed**2) / (2 * max_accel)) if max_accel > 0 else 0
    if accel_steps > steps // 2:
        accel_steps = steps // 2

    if check_limits:
        while poller.poll(0):
            sys.stdin.read(1)
            
    for i in range(steps):
        if check_limits:
            is_manual_pressed = (manual_btn.value() == 0)

            if direction == c['homing_dir'] and (min_pin.value() == 0 or is_manual_pressed):
                print(f"\n Hit MIN Hard Limit (or Manual Override) at step {steps_taken}!")
                break
            
            if direction != c['homing_dir'] and (max_pin.value() == 0 or is_manual_pressed):
                print(f"\n Hit MAX Hard Limit (or Manual Override) at step {steps_taken}!")
                break
                
            if poller.poll(0):
                char = sys.stdin.read(1)
                if char in ['\n', '\r']: 
                    print(f"\n Keyboard limit hit at step {steps_taken}!")
                    break

        # Execute Trapezoidal Speed Calculation
        if i < accel_steps:
            # Acceleration phase
            current_speed = math.sqrt(min_speed**2 + 2 * max_accel * i)
        elif i >= (steps - accel_steps):
            # Deceleration phase
            decel_idx = steps - i
            current_speed = math.sqrt(min_speed**2 + 2 * max_accel * decel_idx)
        else:
            # Cruise phase
            current_speed = max_speed
            
        if current_speed <= 0: current_speed = min_speed
        
        # Convert steps per second to total half-period pulse delay in microseconds
        delay_us = int(1_000_000 / (2 * current_speed))
        
        step_pin.high()
        time.sleep_us(delay_us)
        step_pin.low()
        time.sleep_us(delay_us)
        steps_taken += 1
        
    return steps_taken

def home_motor_hybrid(motor_dict):
    c = motor_dict['config']
    step_pin = motor_dict['step_pin']
    dir_pin = motor_dict['dir_pin']
    min_pin = motor_dict['limit_min_pin']
    max_pin = motor_dict['limit_max_pin']
    
    max_speed = motor_dict["max_speed"]
    max_accel = motor_dict["max_accel"]
    min_speed = 40.0
    
    print(f"\n--- Hybrid Center Homing {c['name']} ---")
    
    def drive_until_trigger(direction, target_pin):
        is_forward = (direction == 1)
        if c['dir_invert'] == -1:
            is_forward = not is_forward
        target_dir_val = 0 if c['homing_dir'] == 0 else 1
        if is_forward:
            target_dir_val = 1 if target_dir_val == 0 else 0
            
        dir_pin.value(target_dir_val)
        steps_taken = 0
        current_speed = min_speed
        
        while poller.poll(0):
            sys.stdin.read(1)
            
        print("   >>> Moving... Waiting for Switch OR Manual Button OR Keyboard 'ENTER' <<<")
        
        while True:
            # Gentle acceleration linear ramp for homing search profile
            if current_speed < (max_speed * 0.5):  # Home at half speed for precision safety
                current_speed += (max_accel * (1.0 / current_speed))
            
            delay_us = int(1_000_000 / (2 * current_speed))
            
            step_pin.high()
            time.sleep_us(delay_us)
            step_pin.low()
            time.sleep_us(delay_us)
            steps_taken += 1
            
            if target_pin.value() == 0:
                return steps_taken, "Hardware Switch"
            if manual_btn.value() == 0:
                return steps_taken, "Manual Push Button"
            if poller.poll(0):
                char = sys.stdin.read(1)
                if char in ['\n', '\r']: 
                    return steps_taken, "Keyboard Simulator"

    # Phase 1: Move to the Left (MIN) Limit
    print("\nPhase 1: Moving to MIN Limit...")
    steps, trigger_type = drive_until_trigger(c['homing_dir'], min_pin)
    print(f"Hit MIN Limit! (Triggered by: {trigger_type})")
    
    print("Backing off switch automatically (200 steps)")
    move_motor(motor_dict, 200, not c['homing_dir'], check_limits=False)

    print("Please release all keys/switches.")
    while min_pin.value() == 0 or manual_btn.value() == 0:
        time.sleep(0.01)
    while poller.poll(0):
        sys.stdin.read(1)

    # Phase 2: Move to the Right (MAX) Limit
    print("\nMoving to MAX Limit and measuring width")
    total_width_steps, trigger_type = drive_until_trigger(not c['homing_dir'], max_pin)
    
    total_width_steps += 200 
    print(f"Hit MAX Limit! (Triggered by: {trigger_type})")
    print(f"Total measured axis width: {total_width_steps} steps.")

    print("Debouncing MAX switch")
    move_motor(motor_dict, 200, c['homing_dir'], check_limits=False)
    
    while max_pin.value() == 0 or manual_btn.value() == 0:
        time.sleep(0.01)
    while poller.poll(0):
        sys.stdin.read(1)

    # Phase 3: Calculate Center and Move Back
    center_steps = (total_width_steps // 2) - 200
    print(f"\nMoving to mechanical center ({center_steps} steps back)")
    move_motor(motor_dict, center_steps, c['homing_dir'], check_limits=True)

    return True

active_idx = 0
current_steps = 5000

def print_dashboard():
    print("\n" * 40)
    print("=========================================================================================")
    print("                            LIVE TMC2209 TUNING DASHBOARD                                ")
    print("=========================================================================================")
    
    for i, m in enumerate(motors):
        marker = ">>" if i == active_idx else "  "
        c = m['config']
        print(f"{marker} [{i}] {c['name']:<9} | Run: {c['run_current_mA']:<4}mA | Max Speed: {m['max_speed']:<5} sps | MIN: GP{c['limit_min']:<2} | MAX: GP{c['limit_max']:<2}")
        
    print("-----------------------------------------------------------------------------------------")
    print(f" Global Test Steps: {current_steps}")
    print("-----------------------------------------------------------------------------------------")
    print(" COMMANDS:")
    print("  [0, 1, 2, 3] Select Active Motor")
    print("  [O] Execute Center Home   [V] Set Max Speed (sps)")
    print("  [S] Set Target Steps      [R] Set Run Current")
    print("  [F] Move Forward (MAX)    [B] Move Backward (MIN)")
    print("  [Q] Quit & Disable All Motors")
    print("=========================================================================================")

try:
    while True:
        print_dashboard()
        cmd = input("Enter command: ").strip().upper()
        
        if cmd in ['0', '1', '2', '3']:
            active_idx = int(cmd)
            
        elif cmd == 'R':
            try:
                val = int(input(f"Enter new RUN current for {motors[active_idx]['name']} (mA): "))
                motors[active_idx]['config']['run_current_mA'] = val
                motors[active_idx]['tmc'].set_motor_current(val, motors[active_idx]['config']['hold_current_mA'])
            except ValueError: pass
            
        elif cmd == 'O':
            home_motor_hybrid(motors[active_idx])
            input("\nPress ENTER to return to dashboard...") 
                
        elif cmd == 'S':
            try:
                current_steps = int(input("Enter new Steps per move: "))
            except ValueError: pass
                
        elif cmd == 'V':
            try:
                val = int(input(f"Enter new Max Speed in sps for {motors[active_idx]['name']}: "))
                motors[active_idx]['max_speed'] = val
                motors[active_idx]['config']['max_speed_sps'] = val
            except ValueError: pass
                
        elif cmd == 'F':
            print(f"\nMoving {motors[active_idx]['name']} FORWARD (Towards MAX)...")
            actual = move_motor(motors[active_idx], current_steps, direction=1)
            if actual < current_steps:
                input("\nPress ENTER to acknowledge emergency stop...")
            
        elif cmd == 'B':
            print(f"\nMoving {motors[active_idx]['name']} BACKWARD (Towards MIN)...")
            actual = move_motor(motors[active_idx], current_steps, direction=0)
            if actual < current_steps:
                input("\nPress ENTER to acknowledge emergency stop...")
            
        elif cmd == 'Q':
            break

except KeyboardInterrupt:
    print("\nDashboard closed by user.")

finally:
    print("Shutting down all motors...")
    for motor in motors:
        motor["tmc"].disable()
    print("Done. Safe to power off.")