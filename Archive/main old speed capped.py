from machine import Pin, UART
import time 
import sys
import select
import math
import micropython
import _thread
from Archive.tmc2209 import TMC2209
import homer

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

poller = select.poll()
poller.register(sys.stdin, select.POLLIN)

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
def motor_stepping_thread(lock, motor_array):
    print("[CORE 1] Motor Engine Spawned and Online.")

    MIN_STEP_INTERVAL_US = 100      # hard floor -> caps top speed, prevents runaway short delays
    LOOP_IDLE_SLEEP_US = 50         # how often we re-evaluate ramps when nothing to step

    # per-motor tick bookkeeping lives outside the lock; only position/target are shared
    last_tick_us = [time.ticks_us() for _ in motor_array]

    while True:
        soonest_wake_us = None
        now = time.ticks_us()

        for idx, m in enumerate(motor_array):
            lock.acquire()
            target = m["target_position"]
            current = m["current_position"]
            lock.release()

            dist = target - current
            dt_s = time.ticks_diff(now, last_tick_us[idx]) / 1_000_000.0
            if dt_s <= 0:
                dt_s = 0.0001
            last_tick_us[idx] = now

            velocity = m["velocity"]          # signed, steps/sec
            max_speed = m["max_speed"]
            max_accel = m["max_accel"]

            if dist == 0:
                # decelerate to a stop instead of slamming velocity to 0
                if velocity > 0:
                    velocity = max(0.0, velocity - max_accel * dt_s)
                elif velocity < 0:
                    velocity = min(0.0, velocity + max_accel * dt_s)
            else:
                direction = 1.0 if dist > 0 else -1.0
                desired_speed = direction * max_speed

                # distance needed to decelerate from current speed to 0
                decel_dist = (velocity * velocity) / (2.0 * max_accel) if max_accel > 0 else 0
                if abs(dist) <= decel_dist and velocity != 0:
                    # close to target -> ramp down toward 0, keep current direction
                    if velocity > 0:
                        velocity = max(0.0, velocity - max_accel * dt_s)
                    else:
                        velocity = min(0.0, velocity + max_accel * dt_s)
                else:
                    # ramp toward max speed in the needed direction
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

# Spawn the motor loop onto Core 1 and pass the global references directly
_thread.start_new_thread(motor_stepping_thread, (position_lock, motors))

# =================================================================
# CORE 0 ENGINE: CONSTANT USB LISTENER (Main Thread)
# =================================================================
serial_buffer = ""
last_debug_time = time.ticks_ms()
packets_received = 0

print("[CORE 0] Serial Listener Waiting for ROS 2...")

try:
    while True:
        bytes_read = 0
        while poller.poll(0) and bytes_read < 128:
            char = sys.stdin.read(1)
            if char:
                serial_buffer += char
                bytes_read += 1
                
        if '>' in serial_buffer:
            end_idx = serial_buffer.rfind('>')
            start_idx = serial_buffer.rfind('<', 0, end_idx)
            
            if start_idx != -1:
                led.toggle()
                packets_received += 1
                clean_str = serial_buffer[start_idx + 1 : end_idx]
                
                if time.ticks_diff(time.ticks_ms(), last_debug_time) > 1000:
                    print(f"BufLen: {len(serial_buffer)} | Parsed: <{clean_str}> | TotalPkts: {packets_received}")
                    last_debug_time = time.ticks_ms()
                
                str_targets = clean_str.split(',')
                
                if len(str_targets) >= len(motors):
                    for i in range(len(motors)):
                        try:
                            target_rad = float(str_targets[i])
                            m_config = motors[i]["config"]
                            steps = (target_rad / (2.0 * math.pi)) * 200 * m_config["microsteps"] * m_config["gear_ratio"]
                            
                            position_lock.acquire()
                            motors[i]["target_position"] = int(steps)
                            position_lock.release()
                        except ValueError:
                            pass 
                            
            serial_buffer = serial_buffer[end_idx + 1:]
            if len(serial_buffer) > 256:
                serial_buffer = ""

except KeyboardInterrupt:
    pass
finally:
    position_lock.acquire()
    for m in motors:
        m["target_position"] = m["current_position"] 
        m["tmc"].disable()
    position_lock.release()
    print("Motors disabled.")