from machine import Pin, UART
import time 
import sys
import select
from tmc2209 import TMC2209

UART_TX_PIN = 0 
UART_RX_PIN = 1 
UART_BAUDRATE = 19200

MOTOR_CONFIGS = [
    {"name": "Base",     "address": 0, "step_pin": 16, "dir_pin": 17, "limit_min": 3, "limit_max": 2, "run_current_mA": 1200, "hold_current_mA": 200, "microsteps": 16, "homing_dir": 0, "delay_us": 100},
    {"name": "Shoulder", "address": 1, "step_pin": 14, "dir_pin": 15, "limit_min": 4, "limit_max": 5, "run_current_mA": 800,  "hold_current_mA": 0,   "microsteps": 16, "homing_dir": 0, "delay_us": 600},
    {"name": "Elbow",    "address": 2, "step_pin": 12, "dir_pin": 13, "limit_min": 6, "limit_max": 7, "run_current_mA": 1000, "hold_current_mA": 200, "microsteps": 16, "homing_dir": 0, "delay_us": 800},
    {"name": "Wrist",    "address": 3, "step_pin": 10, "dir_pin": 11, "limit_min": 8, "limit_max": 9, "run_current_mA": 1000, "hold_current_mA": 200, "microsteps": 16, "homing_dir": 0, "delay_us": 800}
]

poller = select.poll()
poller.register(sys.stdin, select.POLLIN)

uart = UART(0, baudrate=UART_BAUDRATE, tx=Pin(UART_TX_PIN), rx=Pin(UART_RX_PIN))
motors = []

print("Initializing Motors on UART Bus...")
for config in MOTOR_CONFIGS:
    dir_p = Pin(config['dir_pin'], Pin.OUT)
    step_p = Pin(config['step_pin'], Pin.OUT)
    
    limit_min_p = Pin(config['limit_min'], Pin.IN, Pin.PULL_UP) 
    limit_max_p = Pin(config['limit_max'], Pin.IN, Pin.PULL_UP) 
    
    tmc = TMC2209(uart, address=config['address'])
    tmc.clear_gconf()
    tmc.set_motor_current(run_current_ma=config['run_current_mA'], hold_current_ma=config['hold_current_mA'])
    tmc.set_microstepping(config['microsteps'])
    tmc.set_stealthchop_enable(True)
    tmc.enable()
    
    motors.append({
        "name": config['name'],
        "tmc": tmc,
        "step_pin": step_p,
        "dir_pin": dir_p,
        "limit_min_pin": limit_min_p,
        "limit_max_pin": limit_max_p,
        "config": config 
    })

def move_motor(motor_dict, steps, direction, delay_us, check_limits=True):
    c = motor_dict["config"]
    motor_dict["dir_pin"].value(direction)
    step_pin = motor_dict["step_pin"]
    min_pin = motor_dict["limit_min_pin"]
    max_pin = motor_dict["limit_max_pin"]
    
    if check_limits:
        while poller.poll(0):
            sys.stdin.read(1)
            
    steps_taken = 0
    
    for _ in range(steps):
        if check_limits:

            if direction == c['homing_dir'] and min_pin.value() == 0:
                print(f"\n Hit MIN Hard Limit at step {steps_taken}!")
                break
            
            # If moving towards MAX, only check the MAX switch
            if direction != c['homing_dir'] and max_pin.value() == 0:
                print(f"\n Hit MAX Hard Limit at step {steps_taken}!")
                break
                
            # Generic Keyboard E-Stop
            if poller.poll(0):
                char = sys.stdin.read(1)
                if char in ['\n', '\r']: 
                    print(f"\n Keyboard limit hit at step {steps_taken}!")
                    break

        step_pin.high()
        time.sleep_us(delay_us)
        step_pin.low()
        time.sleep_us(delay_us)
        steps_taken += 1
        
    return steps_taken

def home_motor_hybrid(motor_dict, delay_us):
    c = motor_dict['config']
    step_pin = motor_dict['step_pin']
    dir_pin = motor_dict['dir_pin']
    min_pin = motor_dict['limit_min_pin']
    max_pin = motor_dict['limit_max_pin']
    
    print(f"\n--- Hybrid Center Homing {c['name']} ---")
    
    def drive_until_trigger(direction, target_pin):
        dir_pin.value(direction)
        steps_taken = 0
        
        while poller.poll(0):
            sys.stdin.read(1)
            
        print("  >>> Moving... Waiting for Physical Switch OR Keyboard 'ENTER' <<<")
        
        while True:
            step_pin.high()
            time.sleep_us(delay_us)
            step_pin.low()
            time.sleep_us(delay_us)
            steps_taken += 1
            
            if target_pin.value() == 0:
                return steps_taken, "Hardware Switch"
            
            if poller.poll(0):
                char = sys.stdin.read(1)
                if char in ['\n', '\r']: 
                    return steps_taken, "Keyboard Simulator"

    # Phase 1: Move to the Left (MIN) Limit
    print("\nPhase 1: Moving to MIN Limit...")
    steps, trigger_type = drive_until_trigger(c['homing_dir'], min_pin)
    print(f"Hit MIN Limit! (Triggered by: {trigger_type})")
    
    # Notice we can safely leave check_limits=True now, because moving away from MIN ignores the MIN switch!
    print("Backing off switch automatically (200 steps)")
    move_motor(motor_dict, 200, not c['homing_dir'], delay_us)

    print("Please release the key/switch.")
    while min_pin.value() == 0:
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
    move_motor(motor_dict, 200, c['homing_dir'], delay_us)
    while max_pin.value() == 0:
        time.sleep(0.01)
    while poller.poll(0):
        sys.stdin.read(1)

    # Phase 3: Calculate Center and Move Back
    # We already backed off 200 steps, so subtract that from the travel distance
    center_steps = (total_width_steps // 2) - 200
    print(f"\nMoving to mechanical center ({center_steps} steps back)")
    move_motor(motor_dict, center_steps, c['homing_dir'], delay_us)

    return True

active_idx = 0
current_steps = 5000

def print_dashboard():
    print("\n" * 40)
    print("=========================================================================================")
    print("                              LIVE TMC2209 TUNING DASHBOARD                              ")
    print("=========================================================================================")
    
    for i, m in enumerate(motors):
        marker = ">>" if i == active_idx else "  "
        c = m['config']
        print(f"{marker} [{i}] {c['name']:<9} | Run: {c['run_current_mA']:<4}mA | Delay: {c['delay_us']:<4}us | MIN: GP{c['limit_min']:<2} | MAX: GP{c['limit_max']:<2}")
        
    print("-----------------------------------------------------------------------------------------")
    print(f" Global Test Steps: {current_steps}")
    print("-----------------------------------------------------------------------------------------")
    print(" COMMANDS:")
    print("  [0, 1, 2, 3] Select Active Motor")
    print("  [O] Execute Center Home   [D] Set Delay (Speed)")
    print("  [S] Set Steps             [R] Set Run Current")
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
            home_motor_hybrid(motors[active_idx], delay_us=motors[active_idx]['config']['delay_us'])
            input("\nPress ENTER to return to dashboard...") 
                
        elif cmd == 'S':
            try:
                current_steps = int(input("Enter new Steps per move: "))
            except ValueError: pass
                
        elif cmd == 'D':
            try:
                val = int(input(f"Enter new Delay in us for {motors[active_idx]['name']} (Lower = Faster): "))
                motors[active_idx]['config']['delay_us'] = val
            except ValueError: pass
                
        elif cmd == 'F':
            print(f"\nMoving {motors[active_idx]['name']} FORWARD (Towards MAX)...")
            actual = move_motor(motors[active_idx], current_steps, not motors[active_idx]['config']['homing_dir'], motors[active_idx]['config']['delay_us'])
            if actual < current_steps:
                input("\nPress ENTER to acknowledge emergency stop...")
            
        elif cmd == 'B':
            print(f"\nMoving {motors[active_idx]['name']} BACKWARD (Towards MIN)...")
            actual = move_motor(motors[active_idx], current_steps, motors[active_idx]['config']['homing_dir'], motors[active_idx]['config']['delay_us'])
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