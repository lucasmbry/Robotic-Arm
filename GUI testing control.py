from machine import Pin, UART
import time
from tmc2209 import TMC2209

# Shared UART Pins (All 4 drivers connect to these same pins)
UART_TX_PIN = 0 
UART_RX_PIN = 1 
UART_BAUDRATE = 19200

# Ensure each address matches the physical MS1/MS2 pin wiring
MOTOR_CONFIGS = [
    {"name": "Base",     "address": 0, "step_pin": 16, "dir_pin": 17, "run_current_mA": 800, "hold_current_mA": 000, "microsteps": 16},
    {"name": "Shoulder", "address": 1, "step_pin": 14, "dir_pin": 15, "run_current_mA": 800, "hold_current_mA": 300, "microsteps": 16},
    {"name": "Elbow",    "address": 2, "step_pin": 12, "dir_pin": 13, "run_current_mA": 1000, "hold_current_mA": 200, "microsteps": 16},
    {"name": "Wrist",    "address": 3, "step_pin": 10, "dir_pin": 11, "run_current_mA": 1000, "hold_current_mA": 200, "microsteps": 16}
]

# Initialize the shared UART bus
uart = UART(0, baudrate=UART_BAUDRATE, tx=Pin(UART_TX_PIN), rx=Pin(UART_RX_PIN))
motors = []

print("Initializing Motors on UART Bus...")
for config in MOTOR_CONFIGS:
    dir_p = Pin(config['dir_pin'], Pin.OUT)
    step_p = Pin(config['step_pin'], Pin.OUT)
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
        "config": config # Keep a reference to the config dict to track live changes
    })

def move_motor(motor_dict, steps, direction, delay_us):
    motor_dict["dir_pin"].value(direction)
    step_pin = motor_dict["step_pin"]
    for _ in range(steps):
        step_pin.high()
        time.sleep_us(delay_us)
        step_pin.low()
        time.sleep_us(delay_us)

# ==========================================
# INTERACTIVE TERMINAL DASHBOARD
# ==========================================

active_idx = 0
current_steps = 5000
current_delay = 500

def print_dashboard():
    # Print a bunch of newlines to "clear" the Thonny console
    print("\n" * 40)
    print("=========================================")
    print("      LIVE TMC2209 TUNING DASHBOARD      ")
    print("=========================================")
    
    for i, m in enumerate(motors):
        marker = ">>" if i == active_idx else "  "
        c = m['config']
        print(f"{marker} [{i}] {c['name']:<9} | Run: {c['run_current_mA']:<4}mA | Hold: {c['hold_current_mA']:<4}mA")
        
    print("-----------------------------------------")
    print(f" Test Move Settings:  Steps: {current_steps}  |  Delay: {current_delay}us")
    print("-----------------------------------------")
    print(" COMMANDS:")
    print("  [0, 1, 2, 3] Select Active Motor")
    print("  [R] Set Run Current   [H] Set Hold Current")
    print("  [S] Set Steps         [D] Set Delay (Speed)")
    print("  [F] Move Forward      [B] Move Backward")
    print("  [Q] Quit & Disable All Motors")
    print("=========================================")

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
                # Instantly update the physical chip over UART
                motors[active_idx]['tmc'].set_motor_current(val, motors[active_idx]['config']['hold_current_mA'])
            except ValueError:
                pass # Ignored invalid input
                
        elif cmd == 'H':
            try:
                val = int(input(f"Enter new HOLD current for {motors[active_idx]['name']} (mA): "))
                motors[active_idx]['config']['hold_current_mA'] = val
                # Instantly update the physical chip over UART
                motors[active_idx]['tmc'].set_motor_current(motors[active_idx]['config']['run_current_mA'], val)
            except ValueError:
                pass
                
        elif cmd == 'S':
            try:
                current_steps = int(input("Enter new Steps per move: "))
            except ValueError:
                pass
                
        elif cmd == 'D':
            try:
                current_delay = int(input("Enter new Delay in us (Lower = Faster): "))
            except ValueError:
                pass
                
        elif cmd == 'F':
            print(f"\nMoving {motors[active_idx]['name']} FORWARD...")
            move_motor(motors[active_idx], current_steps, 1, current_delay)
            
        elif cmd == 'B':
            print(f"\nMoving {motors[active_idx]['name']} BACKWARD...")
            move_motor(motors[active_idx], current_steps, 0, current_delay)
            
        elif cmd == 'Q':
            break

except KeyboardInterrupt:
    print("\nDashboard closed by user.")

finally:
    for motor in motors:
        motor["tmc"].disable()
