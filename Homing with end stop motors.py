from machine import Pin, UART
import time 
import sys
import select
from tmc2209 import TMC2209

# Shared UART Pins
UART_TX_PIN = 0 
UART_RX_PIN = 1 
UART_BAUDRATE = 19200

MOTOR_CONFIGS = [
    {"name": "Base",     "address": 0, "step_pin": 16, "dir_pin": 17, "limit_pin": 2, "run_current_mA": 800, "hold_current_mA": 200,   "microsteps": 16, "homing_dir": 0, "delay_us": 50},
    {"name": "Shoulder", "address": 1, "step_pin": 14, "dir_pin": 15, "limit_pin": 3, "run_current_mA": 800, "hold_current_mA": 0,   "microsteps": 16, "homing_dir": 0, "delay_us": 600},
    {"name": "Elbow",    "address": 2, "step_pin": 12, "dir_pin": 13, "limit_pin": 4, "run_current_mA": 1000, "hold_current_mA": 200, "microsteps": 16, "homing_dir": 0, "delay_us": 800},
    {"name": "Wrist",    "address": 3, "step_pin": 10, "dir_pin": 11, "limit_pin": 5, "run_current_mA": 1000, "hold_current_mA": 200, "microsteps": 16, "homing_dir": 0, "delay_us": 800}
]

# Set up a non-blocking keyboard listener
poller = select.poll()
poller.register(sys.stdin, select.POLLIN)

# Initialize the shared UART bus
uart = UART(0, baudrate=UART_BAUDRATE, tx=Pin(UART_TX_PIN), rx=Pin(UART_RX_PIN))
motors = []

print("Initializing Motors on UART Bus...")
for config in MOTOR_CONFIGS:
    dir_p = Pin(config['dir_pin'], Pin.OUT)
    step_p = Pin(config['step_pin'], Pin.OUT)
    limit_p = Pin(config['limit_pin'], Pin.IN, Pin.PULL_UP) 
    
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
        "limit_pin": limit_p,
        "config": config 
    })

def move_motor(motor_dict, steps, direction, delay_us):
    motor_dict["dir_pin"].value(direction)
    step_pin = motor_dict["step_pin"]
    for _ in range(steps):
        step_pin.high()
        time.sleep_us(delay_us)
        step_pin.low()
        time.sleep_us(delay_us)

def home_motor_simulated(motor_dict, delay_us):
    c = motor_dict['config']
    step_pin = motor_dict['step_pin']
    dir_pin = motor_dict['dir_pin']
    
    print(f"\n--- Simulated Center Homing {c['name']} ---")
    
    # Helper function to step until the ENTER key is pressed
    def drive_until_keypress(direction):
        dir_pin.value(direction)
        steps_taken = 0
        
        # Flush any accidental keyboard mashing before we start moving
        while poller.poll(0):
            sys.stdin.read(1)
            
        print("  >>> Motor is moving... Press ENTER to simulate switch click! <<<")
        
        while True:
            step_pin.high()
            time.sleep_us(delay_us)
            step_pin.low()
            time.sleep_us(delay_us)
            steps_taken += 1
            
            # Non-blocking check to see if the user pressed a key
            if poller.poll(0):
                char = sys.stdin.read(1)
                if char in ['\n', '\r']: # Look for the Enter key
                    return steps_taken

    # Phase 1: Move to the Left Limit
    print("\nPhase 1: Moving to Endstop 1 (Left)...")
    drive_until_keypress(c['homing_dir'])
    print("✅ *CLICK* Hit Endstop 1!")
    
    # Simulate backing off the physical switch safely
    print("Backing off switch automatically (200 steps)...")
    move_motor(motor_dict, 200, not c['homing_dir'], delay_us)

    # Phase 2: Move to the Right Limit and Count
    print("\nPhase 2: Moving to Endstop 2 (Right) and measuring width...")
    total_width_steps = drive_until_keypress(not c['homing_dir'])
    
    # Add back the 200 steps we used to back off, so the total width is perfectly accurate
    total_width_steps += 200 
    print(f"✅ *CLICK* Hit Endstop 2! Total axis width: {total_width_steps} steps.")

    # Phase 3: Calculate Center and Move Back
    center_steps = total_width_steps // 2
    print(f"\nPhase 3: Moving to mechanical center ({center_steps} steps back)...")
    move_motor(motor_dict, center_steps, c['homing_dir'], delay_us)

    print("🎯 Center Homing Complete! Arm is perfectly aligned.")
    return True

# ==========================================
# INTERACTIVE TERMINAL DASHBOARD
# ==========================================

active_idx = 0
current_steps = 5000

def print_dashboard():
    print("\n" * 40)
    print("=========================================================================")
    print("                      LIVE TMC2209 TUNING DASHBOARD                      ")
    print("=========================================================================")
    
    for i, m in enumerate(motors):
        marker = ">>" if i == active_idx else "  "
        c = m['config']
        print(f"{marker} [{i}] {c['name']:<9} | Run: {c['run_current_mA']:<4}mA | Hold: {c['hold_current_mA']:<4}mA | Delay: {c['delay_us']:<4}us")
        
    print("-------------------------------------------------------------------------")
    print(f" Global Test Steps: {current_steps}")
    print("-------------------------------------------------------------------------")
    print(" COMMANDS:")
    print("  [0, 1, 2, 3] Select Active Motor")
    print("  [R] Set Run Current     [H] Set Hold Current")
    print("  [O] Execute Center Home [D] Set Delay (Speed)")
    print("  [S] Set Steps")
    print("  [F] Move Forward        [B] Move Backward")
    print("  [Q] Quit & Disable All Motors")
    print("=========================================================================")

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
                
        elif cmd == 'H':
            try:
                val = int(input(f"Enter new HOLD current for {motors[active_idx]['name']} (mA): "))
                motors[active_idx]['config']['hold_current_mA'] = val
                motors[active_idx]['tmc'].set_motor_current(motors[active_idx]['config']['run_current_mA'], val)
            except ValueError: pass
            
        elif cmd == 'O':
            # Run the keyboard-simulated homing routine
            home_motor_simulated(motors[active_idx], delay_us=motors[active_idx]['config']['delay_us'])
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
            print(f"\nMoving {motors[active_idx]['name']} FORWARD at {motors[active_idx]['config']['delay_us']}us delay...")
            move_motor(motors[active_idx], current_steps, 1, motors[active_idx]['config']['delay_us'])
            
        elif cmd == 'B':
            print(f"\nMoving {motors[active_idx]['name']} BACKWARD at {motors[active_idx]['config']['delay_us']}us delay...")
            move_motor(motors[active_idx], current_steps, 0, motors[active_idx]['config']['delay_us'])
            
        elif cmd == 'Q':
            break

except KeyboardInterrupt:
    print("\nDashboard closed by user.")

finally:
    print("Shutting down all motors...")
    for motor in motors:
        motor["tmc"].disable()
    print("Done. Safe to power off.")