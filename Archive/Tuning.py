from machine import Pin, UART
import time 
import sys
import math
import micropython
import _thread
from Archive.tmc2209 import TMC2209

# --- SAFETY WINDOW ---
print("Booting... 3 second safety window active.")
time.sleep(3)
micropython.kbd_intr(-1)

# --- 1. HARDWARE & MOTOR SETUP ---
master_override_button = Pin(15, Pin.IN, Pin.PULL_UP)
led = Pin("LED", Pin.OUT)  

UART_TX_PIN = 0 
UART_RX_PIN = 1 
UART_BAUDRATE = 19200

MOTOR_CONFIGS = [
    {"name": "Base",     "address": 0, "step_pin": 17, "dir_pin": 16, "limit_min": 10, "limit_max": 11, "run_current_mA": 800,  "hold_current_mA": 100, "microsteps": 8,  "homing_dir": 0, "gear_ratio": 15.0, "dir_invert": 1, "max_speed_sps": 4000, "max_accel_sps2": 8000},
    {"name": "Shoulder", "address": 1, "step_pin": 12, "dir_pin": 13, "limit_min": 4, "limit_max": 5, "run_current_mA": 800,  "hold_current_mA": 100, "microsteps": 8,  "homing_dir": 0, "gear_ratio": 40.0, "dir_invert": -1, "max_speed_sps": 4000, "max_accel_sps2": 8000},
    {"name": "Elbow",    "address": 2, "step_pin": 19, "dir_pin": 18, "limit_min": 6, "limit_max": 7, "run_current_mA": 800,  "hold_current_mA": 100, "microsteps": 8,  "homing_dir": 0, "gear_ratio": 20.0, "dir_invert": 1, "max_speed_sps": 4000, "max_accel_sps2": 8000},
    {"name": "Wrist",    "address": 3, "step_pin": 21, "dir_pin": 20, "limit_min": 8, "limit_max": 8, "run_current_mA": 800,  "hold_current_mA": 100, "microsteps": 8,  "homing_dir": 0, "gear_ratio": 1.0, "dir_invert": 1, "max_speed_sps": 4000, "max_accel_sps2": 8000},
]

uart = UART(0, baudrate=UART_BAUDRATE, tx=Pin(UART_TX_PIN), rx=Pin(UART_RX_PIN))
motors = []

print("Initializing Drivers...")
for config in MOTOR_CONFIGS:
    dir_p = Pin(config['dir_pin'], Pin.OUT)
    step_p = Pin(config['step_pin'], Pin.OUT)
    
    tmc = TMC2209(uart, address=config['address'])
    tmc.clear_gconf()
    gconf = tmc.read_int(tmc.REG_GCONF)
    tmc.write_reg(tmc.REG_GCONF, gconf | (1 << 6) | (1 << 7))
    
    tmc.set_motor_current(run_current_ma=config['run_current_mA'], hold_current_ma=config['hold_current_mA'])
    tmc.set_microstepping(config['microsteps'])
    tmc.set_stealthchop_enable(True) 
    tmc.enable()
    
    motors.append({
        "config": config,
        "tmc": tmc,
        "step_pin_obj": step_p,
        "dir_pin_obj": dir_p,
        "current_position": 0,
        "target_position": 0,
        "homing_dir_val": config['homing_dir'],
        "dir_invert": config['dir_invert'],
        "max_speed": config['max_speed_sps'],
        "max_accel": config['max_accel_sps2'],
        "velocity": 0.0,
        "next_step_due_us": time.ticks_us(),
    })

# --- 2. GLOBAL SYSTEM THREAD LOCKS ---
position_lock = _thread.allocate_lock()

# =================================================================
# CORE 1 ENGINE: TRAPEZOIDAL-RAMPED MOTOR STEPPING
# =================================================================
@micropython.native
def motor_stepping_thread(lock, motor_array):
    import time  
    
    print("[CORE 1] Motor Engine Spawned and Online.")
    MIN_STEP_INTERVAL_US = 10
    LOOP_IDLE_SLEEP_US = 50
    last_tick_us = [time.ticks_us() for _ in motor_array]

    while True:
        soonest_wake_us = None
        now = time.ticks_us()

        for idx, m in enumerate(motor_array):
            lock.acquire()
            target = m["target_position"]
            current = m["current_position"]
            max_speed = m["max_speed"]  
            max_accel = m["max_accel"]  
            lock.release()

            dist = target - current
            dt_s = time.ticks_diff(now, last_tick_us[idx]) / 1_000_000.0
            if dt_s <= 0:
                dt_s = 0.0001
            last_tick_us[idx] = now

            velocity = m["velocity"]

            if dist == 0:
                if velocity > 0:
                    velocity = max(0.0, velocity - max_accel * dt_s)
                elif velocity < 0:
                    velocity = min(0.0, velocity + max_accel * dt_s)
            else:
                direction = 1.0 if dist > 0 else -1.0
                desired_speed = direction * max_speed

                decel_dist = (velocity * velocity) / (2.0 * max_accel) if max_accel > 0 else 0
                if abs(dist) <= decel_dist and velocity != 0:
                    if velocity > 0:
                        velocity = max(0.0, velocity - max_accel * dt_s)
                    else:
                        velocity = min(0.0, velocity + max_accel * dt_s)
                else:
                    if velocity < desired_speed:
                        velocity = min(desired_speed, velocity + max_accel * dt_s)
                    elif velocity > desired_speed:
                        velocity = max(desired_speed, velocity - max_accel * dt_s)

            m["velocity"] = velocity

            if velocity != 0 and dist != 0:
                interval_us = int(1_000_000.0 / abs(velocity))
                if interval_us < MIN_STEP_INTERVAL_US:
                    interval_us = MIN_STEP_INTERVAL_US

                if time.ticks_diff(now, m["next_step_due_us"]) >= 0:
                    is_forward = (velocity > 0)
                    if m["dir_invert"] == -1:
                        is_forward = not is_forward
                    m["dir_pin_obj"].value(not m["homing_dir_val"] if is_forward else m["homing_dir_val"])

                    m["step_pin_obj"].high()
                    time.sleep_us(2)
                    m["step_pin_obj"].low()

                    lock.acquire()
                    if velocity > 0:
                        m["current_position"] += 1
                    else:
                        m["current_position"] -= 1
                    lock.release()

                    m["next_step_due_us"] = time.ticks_add(now, interval_us)

                wake = time.ticks_diff(m["next_step_due_us"], now)
                if soonest_wake_us is None or wake < soonest_wake_us:
                    soonest_wake_us = wake

        sleep_for = soonest_wake_us if soonest_wake_us and soonest_wake_us > 0 else LOOP_IDLE_SLEEP_US
        time.sleep_us(min(sleep_for, LOOP_IDLE_SLEEP_US) if sleep_for else LOOP_IDLE_SLEEP_US)

_thread.start_new_thread(motor_stepping_thread, (position_lock, motors))

# =================================================================
# CORE 0 ENGINE: INTERACTIVE CLI TUNER
# =================================================================
print("\n--- MOTOR TUNING CLI ---")
print("Commands:")
print("  status        - Show positions, speeds, and accels of all motors")
print("  sel <id>      - Select active motor to tune (0 to 3)")
print("  move <steps>  - Move active motor by relative steps (e.g., move -1000)")
print("  speed <val>   - Change max speed (e.g., speed 6000)")
print("  accel <val>   - Change max acceleration (e.g., accel 10000)")
print("------------------------\n")

active_id = 0

try:
    while True:
        # Block and wait for user input from standard REPL
        raw_input = input(f"\n[Motor {active_id} - {motors[active_id]['config']['name']}] > ").strip().lower()
        if not raw_input:
            continue
            
        args = raw_input.split()
        cmd = args[0]
        
        if cmd == "status":
            print(f"{'ID':<4} | {'Name':<10} | {'Pos':<8} | {'Target':<8} | {'Speed':<8} | {'Accel':<8}")
            print("-" * 55)
            for i, m in enumerate(motors):
                print(f"{i:<4} | {m['config']['name']:<10} | {m['current_position']:<8} | {m['target_position']:<8} | {m['max_speed']:<8} | {m['max_accel']:<8}")
                
        elif cmd == "sel":
            if len(args) > 1 and args[1].isdigit():
                idx = int(args[1])
                if 0 <= idx < len(motors):
                    active_id = idx
                else:
                    print("Invalid motor ID. Must be 0-3.")
            else:
                print("Usage: sel <id>")
                
        elif cmd == "move":
            if len(args) > 1:
                try:
                    steps = int(args[1])
                    position_lock.acquire()
                    motors[active_id]["target_position"] += steps
                    position_lock.release()
                    print(f"Added {steps} steps. New target: {motors[active_id]['target_position']}")
                except ValueError:
                    print("Error: Steps must be an integer.")
            else:
                print("Usage: move <steps>")
                
        elif cmd == "speed":
            if len(args) > 1:
                try:
                    spd = int(args[1])
                    position_lock.acquire()
                    motors[active_id]["max_speed"] = spd
                    position_lock.release()
                    print(f"Max speed for {motors[active_id]['config']['name']} set to {spd} sps.")
                except ValueError:
                    print("Error: Speed must be an integer.")
            else:
                print("Usage: speed <val>")
                
        elif cmd == "accel":
            if len(args) > 1:
                try:
                    acc = int(args[1])
                    position_lock.acquire()
                    motors[active_id]["max_accel"] = acc
                    position_lock.release()
                    print(f"Max accel for {motors[active_id]['config']['name']} set to {acc} sps^2.")
                except ValueError:
                    print("Error: Acceleration must be an integer.")
                
        else:
            print("Unknown command. Valid commands: status, sel, move, speed, accel")

except KeyboardInterrupt:
    print("\nExiting tuning mode...")
finally:
    position_lock.acquire()
    for m in motors:
        m["target_position"] = m["current_position"] 
        m["tmc"].disable()
    position_lock.release()
    print("Motors safely disabled.")