from machine import Pin, UART, PWM
import time 
import sys
import select
import math
import micropython
from tmc2209 import TMC2209

micropython.kbd_intr(-1)

# Hardware setup for 2 UART buses
UART0_TX_PIN = 0 
UART0_RX_PIN = 1 
UART1_TX_PIN = 4 
UART1_RX_PIN = 5 
UART_BAUDRATE = 19200

# ---------------------------------------------------------
# SHIFT REGISTER SETUP (For DIR pins)
# ---------------------------------------------------------
sr_data = Pin(26, Pin.OUT)
sr_clock = Pin(27, Pin.OUT)
sr_latch = Pin(28, Pin.OUT)
global_dir_state = 0b00000000 

def update_directions():
    sr_latch.value(0)
    for i in range(7, -1, -1):
        sr_clock.value(0)
        bit = (global_dir_state >> i) & 1
        sr_data.value(bit)
        sr_clock.value(1)
    sr_latch.value(1)

def set_motor_direction(motor_index, is_forward):
    global global_dir_state
    if is_forward:
        global_dir_state |= (1 << motor_index)
    else:
        global_dir_state &= ~(1 << motor_index)
    update_directions()

# ---------------------------------------------------------
# SERVO SETUP (GP22)
# ---------------------------------------------------------
servo = PWM(Pin(22))
servo.freq(50)

def set_servo_angle(angle):
    if angle < 0: angle = 0
    if angle > 180: angle = 180
    duty = int(3276 + (angle / 180.0) * (6553 - 3276))
    servo.duty_u16(duty)

# Ensure servo starts in a known position
set_servo_angle(90)

# ---------------------------------------------------------
# MOTOR CONFIGURATIONS
# ---------------------------------------------------------
# Note: 'dir_pin' is replaced by 'shift_idx' (Q0-Q5 on the 74HC595).
# 'step_pin' uses GP16-GP21. Limit switches use GP2,3,6-15.
MOTOR_CONFIGS = [
    {"name": "1st Axis", "uart_bus": 0, "address": 0, "shift_idx": 0, "step_pin": 16, "limit_min": 6,  "limit_max": 7,  "run_current_mA": 800,  "hold_current_mA": 300, "microsteps": 8, "homing_dir": 0, "gear_ratio": 15.0, "dir_invert": 1, "max_speed_sps": 2000, "max_accel_sps2": 8000},
    {"name": "2nd Axis", "uart_bus": 0, "address": 1, "shift_idx": 1, "step_pin": 17, "limit_min": 8,  "limit_max": 9,  "run_current_mA": 1500, "hold_current_mA": 500, "microsteps": 8, "homing_dir": 0, "gear_ratio": 40.0, "dir_invert": 1, "max_speed_sps": 5000, "max_accel_sps2": 8000},
    {"name": "3rd Axis", "uart_bus": 0, "address": 2, "shift_idx": 2, "step_pin": 18, "limit_min": 10, "limit_max": 11, "run_current_mA": 800,  "hold_current_mA": 300, "microsteps": 8, "homing_dir": 0, "gear_ratio": 20.0, "dir_invert": -1, "max_speed_sps": 3000, "max_accel_sps2": 8000},
    {"name": "4th Axis", "uart_bus": 0, "address": 3, "shift_idx": 3, "step_pin": 19, "limit_min": 12, "limit_max": 13, "run_current_mA": 800,  "hold_current_mA": 300, "microsteps": 8, "homing_dir": 0, "gear_ratio": 5.0,  "dir_invert": 1, "max_speed_sps": 5000, "max_accel_sps2": 8000},
    {"name": "5th Axis", "uart_bus": 1, "address": 0, "shift_idx": 4, "step_pin": 20, "limit_min": 14, "limit_max": 15, "run_current_mA": 800,  "hold_current_mA": 300, "microsteps": 8, "homing_dir": 0, "gear_ratio": 3.0,  "dir_invert": -1, "max_speed_sps": 3000, "max_accel_sps2": 8000},
    {"name": "6th Axis", "uart_bus": 1, "address": 1, "shift_idx": 5, "step_pin": 21, "limit_min": 3,  "limit_max": 2,  "run_current_mA": 800,  "hold_current_mA": 300, "microsteps": 8, "homing_dir": 0, "gear_ratio": 1.0,  "dir_invert": 1, "max_speed_sps": 5000, "max_accel_sps2": 8000},
]

poller = select.poll()
poller.register(sys.stdin, select.POLLIN)

uart0 = UART(0, baudrate=UART_BAUDRATE, tx=Pin(UART0_TX_PIN), rx=Pin(UART0_RX_PIN))
uart1 = UART(1, baudrate=UART_BAUDRATE, tx=Pin(UART1_TX_PIN), rx=Pin(UART1_RX_PIN))
uarts = [uart0, uart1]
motors = []

print("Drivers starting")
update_directions() # Push initial zero state to shift register

for config in MOTOR_CONFIGS:
    step_p = Pin(config['step_pin'], Pin.OUT)
    limit_min_p = Pin(config['limit_min'], Pin.IN, Pin.PULL_UP) 
    limit_max_p = Pin(config['limit_max'], Pin.IN, Pin.PULL_UP) 
    
    active_uart = uarts[config['uart_bus']]
    tmc = TMC2209(active_uart, address=config['address'])
    tmc.clear_gconf()

    gconf = tmc.read_int(tmc.REG_GCONF)
    tmc.write_reg(tmc.REG_GCONF, gconf | (1 << 6) | (1 << 7))
    tmc.set_motor_current(run_current_ma=config['run_current_mA'], hold_current_ma=config['hold_current_mA'])
    tmc.set_microstepping(config['microsteps'])
    tmc.set_stealthchop_enable(False)
    tmc.enable()
    
    motors.append({
        "name": config['name'],
        "tmc": tmc,
        "step_pin": step_p,
        "shift_idx": config['shift_idx'],
        "limit_min_pin": limit_min_p,
        "limit_max_pin": limit_max_p,
        "config": config,
        "max_speed": config['max_speed_sps'],
        "max_accel": config['max_accel_sps2']
    })

def move_motor(motor_dict, steps, direction, check_limits=True):
    c = motor_dict["config"]
    is_forward = (direction == 1)
    if c['dir_invert'] == -1:
        is_forward = not is_forward
    target_dir_val = 0 if c['homing_dir'] == 0 else 1
    if is_forward:
        target_dir_val = 1 if target_dir_val == 0 else 0
        
    # Update via Shift Register instead of direct pin
    set_motor_direction(motor_dict["shift_idx"], target_dir_val)
    
    step_pin = motor_dict["step_pin"]
    min_pin = motor_dict["limit_min_pin"]
    max_pin = motor_dict["limit_max_pin"]
    
    max_speed = motor_dict["max_speed"]
    max_accel = motor_dict["max_accel"]
    min_speed = 40.0 
    current_speed = min_speed
    steps_taken = 0
    
    accel_steps = int((max_speed**2 - min_speed**2) / (2 * max_accel)) if max_accel > 0 else 0
    if accel_steps > steps // 2:
        accel_steps = steps // 2

    if check_limits:
        while poller.poll(0):
            sys.stdin.read(1)
            
    for i in range(steps):
        if check_limits:
            if direction == c['homing_dir'] and min_pin.value() == 0:
                print(f"\n Hit MIN Hard Limit at step {steps_taken}!")
                break
            if direction != c['homing_dir'] and max_pin.value() == 0:
                print(f"\n Hit MAX Hard Limit at step {steps_taken}!")
                break
                
            if poller.poll(0):
                char = sys.stdin.read(1)
                if char in ['\n', '\r']: 
                    print(f"\n Keyboard limit hit at step {steps_taken}!")
                    break

        if i < accel_steps:
            current_speed = math.sqrt(min_speed**2 + 2 * max_accel * i)
        elif i >= (steps - accel_steps):
            decel_idx = steps - i
            current_speed = math.sqrt(min_speed**2 + 2 * max_accel * decel_idx)
        else:
            current_speed = max_speed
            
        if current_speed <= 0: current_speed = min_speed
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
    min_pin = motor_dict['limit_min_pin']
    max_pin = motor_dict['limit_max_pin']
    shift_idx = motor_dict['shift_idx']
    
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
            
        set_motor_direction(shift_idx, target_dir_val)
        steps_taken = 0
        current_speed = min_speed
        
        while poller.poll(0):
            sys.stdin.read(1)
            
        print("   >>> Moving... Waiting for Switch OR Keyboard 'ENTER' <<<")
        
        while True:
            if current_speed < (max_speed * 0.5):
                current_speed += (max_accel * (1.0 / current_speed))
            
            delay_us = int(1_000_000 / (2 * current_speed))
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

    print("\nPhase 1: Moving to MIN Limit...")
    steps, trigger_type = drive_until_trigger(c['homing_dir'], min_pin)
    print(f"Hit MIN Limit! (Triggered by: {trigger_type})")
    
    print("Backing off switch automatically (200 steps)")
    move_motor(motor_dict, 200, not c['homing_dir'], check_limits=False)

    print("Please release all switches.")
    while min_pin.value() == 0:
        time.sleep(0.01)
    while poller.poll(0):
        sys.stdin.read(1)

    print("\nMoving to MAX Limit and measuring width")
    total_width_steps, trigger_type = drive_until_trigger(not c['homing_dir'], max_pin)
    total_width_steps += 200 
    
    print(f"Hit MAX Limit! (Triggered by: {trigger_type})")
    print(f"Total measured axis width: {total_width_steps} steps.")

    print("Debouncing MAX switch")
    move_motor(motor_dict, 200, c['homing_dir'], check_limits=False)
    
    while max_pin.value() == 0:
        time.sleep(0.01)
    while poller.poll(0):
        sys.stdin.read(1)

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
        print(f"{marker} [{i}] {c['name']:<9} | Run: {c['run_current_mA']:<4}mA | Speed: {m['max_speed']:<4} sps | Accel: {m['max_accel']:<5} sps²")
        
    print("-----------------------------------------------------------------------------------------")
    print(f" Global Test Steps: {current_steps}")
    print("-----------------------------------------------------------------------------------------")
    print(" COMMANDS:")
    print("  [0, 1, 2, 3, 4, 5] Select Active Motor")
    print("  [O] Execute Center Home   [H] Home ALL (Reverse)   [V] Set Max Speed (sps)")
    print("  [A] Set Acceleration      [S] Set Target Steps     [R] Set Run Current")
    print("  [F] Move Forward (MAX)    [B] Move Backward (MIN)  [G] Set Servo Angle (0-180)")
    print("  [Q] Quit & Disable All Motors")
    print("=========================================================================================")

try:
    while True:
        print_dashboard()
        cmd = input("Enter command: ").strip().upper()
        
        if cmd in ['0', '1', '2', '3', '4', '5']:
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
                
        elif cmd == 'H':
            print("\nHoming all motors in reverse order (5 -> 0)...")
            # Loop from 5 down to 0
            for i in range(len(motors) - 1, -1, -1):
                print(f"\n--- Starting Auto-Home for {motors[i]['name']} ---")
                home_motor_hybrid(motors[i])
            input("\nAll motors homed! Press ENTER to return to dashboard...")
                
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

        elif cmd == 'A':
            try:
                val = int(input(f"Enter new Acceleration in sps² for {motors[active_idx]['name']}: "))
                motors[active_idx]['max_accel'] = val
                motors[active_idx]['config']['max_accel_sps2'] = val
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
                
        elif cmd == 'G':
            try:
                val = int(input("Enter target Servo Angle (0-180): "))
                set_servo_angle(val)
            except ValueError: pass
            
        elif cmd == 'Q':
            break

except KeyboardInterrupt:
    print("\nDashboard closed by user.")

finally:
    print("Shutting down all motors...")
    for motor in motors:
        motor["tmc"].disable()
    print("Done. Safe to power off.")