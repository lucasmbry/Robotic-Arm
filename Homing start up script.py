from machine import Pin, UART
import time 
from tmc2209 import TMC2209

# Shared UART Pins
UART_TX_PIN = 0 
UART_RX_PIN = 1 
UART_BAUDRATE = 19200
maximum_homing_steps = 1000000

# Added: "delay_us" to each config so every motor can have a unique speed
MOTOR_CONFIGS = [
    {"name": "Base",     "address": 0, "step_pin": 16, "dir_pin": 17, "diag_pin": 2, "run_current_mA": 800, "hold_current_mA": 200,   "microsteps": 16, "sg_sensitivity": 50, "homing_dir": 0, "delay_us": 200},
    {"name": "Shoulder", "address": 1, "step_pin": 14, "dir_pin": 15, "diag_pin": 3, "run_current_mA": 800, "hold_current_mA": 0, "microsteps": 16, "sg_sensitivity": 5, "homing_dir": 0, "delay_us": 600},
    {"name": "Elbow",    "address": 2, "step_pin": 12, "dir_pin": 13, "diag_pin": 4, "run_current_mA": 1000, "hold_current_mA": 200, "microsteps": 16, "sg_sensitivity": 100, "homing_dir": 0, "delay_us": 800},
    {"name": "Wrist",    "address": 3, "step_pin": 10, "dir_pin": 11, "diag_pin": 5, "run_current_mA": 1000, "hold_current_mA": 200, "microsteps": 16, "sg_sensitivity": 100, "homing_dir": 0, "delay_us": 800}
]

# Initialize the shared UART bus
uart = UART(0, baudrate=UART_BAUDRATE, tx=Pin(UART_TX_PIN), rx=Pin(UART_RX_PIN))
motors = []

print("Initializing Motors on UART Bus...")
for config in MOTOR_CONFIGS:
    dir_p = Pin(config['dir_pin'], Pin.OUT)
    step_p = Pin(config['step_pin'], Pin.OUT)
    diag_p = Pin(config['diag_pin'], Pin.IN, Pin.PULL_DOWN) 
    
    tmc = TMC2209(uart, address=config['address'])
    
    tmc.clear_gconf()
    tmc.set_motor_current(run_current_ma=config['run_current_mA'], hold_current_ma=config['hold_current_mA'])
    tmc.set_microstepping(config['microsteps'])
    tmc.set_stealthchop_enable(True)
    
    # Enable StallGuard feature
    tmc.set_tcoolthrs(0xFFFFF) 
    tmc.set_sgthrs(config['sg_sensitivity'])
    
    tmc.enable()
    
    motors.append({
        "name": config['name'],
        "tmc": tmc,
        "step_pin": step_p,
        "dir_pin": dir_p,
        "diag_pin": diag_p,
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

def home_motor(motor_dict, max_steps, delay_us):
    c = motor_dict['config']
    tmc = motor_dict['tmc']
    step_pin = motor_dict['step_pin']
    diag_pin = motor_dict['diag_pin']
    
    # Ensure current sensitivity is pushed to the chip before homing
    tmc.set_sgthrs(c['sg_sensitivity'])
    motor_dict["dir_pin"].value(c['homing_dir'])
    
    print(f"\n--- Homing {c['name']} ---")
    print("Watching for StallGuard DIAG trigger...")
    
    consecutive_stalls = 0
    REQUIRED_STALLS = 8
    IGNORE_STEPS = 1000 # <-- THE MAGIC FIX: Ignore the first 200 steps!
    
    for step in range(max_steps):
        step_pin.high()
        time.sleep_us(delay_us)
        step_pin.low()
        time.sleep_us(delay_us)
        
        # Only start listening to the DIAG pin AFTER the motor has stabilized!
        if step > IGNORE_STEPS:
            if diag_pin.value() == 1:
                consecutive_stalls += 1
                
                if consecutive_stalls >= REQUIRED_STALLS:
                    print(f"\n✅ TRUE STALL DETECTED at step {step}!")
                    print("Backing off hard stop...")
                    motor_dict["dir_pin"].value(not c['homing_dir'])
                    for _ in range(200):
                        step_pin.high()
                        time.sleep_us(delay_us)
                        step_pin.low()
                        time.sleep_us(delay_us)
                    return True
            else:
                consecutive_stalls = 0
            
        # Print live data every 100 steps
        if step % 100 == 0:
            live_sg = tmc.get_sg_result()
            print(f"Moving... Live SG_RESULT: {live_sg} (Stall Counter: {consecutive_stalls})")

    print("\n❌ WARNING: Reached max steps without stalling!")
    return False

# ==========================================
# INTERACTIVE TERMINAL DASHBOARD
# ==========================================

active_idx = 0
current_steps = 5000
# Removed global current_delay, it is now pulled from the motor config!

def print_dashboard():
    print("\n" * 40)
    print("=========================================================================")
    print("                      LIVE TMC2209 TUNING DASHBOARD                      ")
    print("=========================================================================")
    
    for i, m in enumerate(motors):
        marker = ">>" if i == active_idx else "  "
        c = m['config']
        print(f"{marker} [{i}] {c['name']:<9} | Run: {c['run_current_mA']:<4}mA | Hold: {c['hold_current_mA']:<4}mA | Sens: {c['sg_sensitivity']:<3} | Delay: {c['delay_us']:<4}us")
        
    print("-------------------------------------------------------------------------")
    print(f" Global Test Steps: {current_steps}")
    print("-------------------------------------------------------------------------")
    print(" COMMANDS:")
    print("  [0, 1, 2, 3] Select Active Motor")
    print("  [R] Set Run Current     [H] Set Hold Current")
    print("  [V] Set SG Sensitivity  [D] Set Delay (Speed)")
    print("  [O] Execute Homing      [S] Set Steps")
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
            
        elif cmd == 'V':
            try:
                val = int(input(f"Enter new SG Sensitivity for {motors[active_idx]['name']} (0-255): "))
                motors[active_idx]['config']['sg_sensitivity'] = min(255, max(0, val))
                motors[active_idx]['tmc'].set_sgthrs(motors[active_idx]['config']['sg_sensitivity'])
            except ValueError: pass
            
        elif cmd == 'O':
            print(f"\nExecuting Sensorless Homing for {motors[active_idx]['name']}...")
            # Now uses the motor's specific delay
            home_motor(motors[active_idx], max_steps=maximum_homing_steps, delay_us=motors[active_idx]['config']['delay_us'])
            time.sleep(2) 
                
        elif cmd == 'S':
            try:
                current_steps = int(input("Enter new Steps per move: "))
            except ValueError: pass
                
        elif cmd == 'D':
            try:
                val = int(input(f"Enter new Delay in us for {motors[active_idx]['name']} (Lower = Faster): "))
                # Updates the delay variable for this specific motor
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