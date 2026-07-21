from machine import Pin, UART
import time 
import sys
import select
import math
from tmc2209 import TMC2209
import homer

master_override_button = Pin(15, Pin.IN, Pin.PULL_UP)

UART_TX_PIN = 0 
UART_RX_PIN = 1 
UART_BAUDRATE = 19200

# Control loop timing. 50Hz = 20,000 microseconds. 
# Adjust this to match your ROS 2 controller update rate.
CONTROL_PERIOD_US = 20000
MIN_STEP_DELAY_US = 5  # Min 5µs = 200kHz max stepping speed 

MOTOR_CONFIGS = [
    {"name": "Base",     "address": 0, "step_pin": 17, "dir_pin": 16, "limit_min": 22, "limit_max": 14, "run_current_mA": 800,  "hold_current_mA": 150, "microsteps": 2,  "homing_dir": 0, "delay_us": 100, "home_offset": 0, "gear_ratio": 15.0, "dir_invert": 1},
    {"name": "Shoulder", "address": 1, "step_pin": 12, "dir_pin": 13, "limit_min": 5, "limit_max": 4, "run_current_mA": 500,  "hold_current_mA": 150, "microsteps": 2,  "homing_dir": 0, "delay_us": 100, "home_offset": -11732, "gear_ratio": 40.0, "dir_invert": 1}, 
    {"name": "Elbow", "address": 2, "step_pin": 19, "dir_pin": 18, "limit_min": 5, "limit_max": 4, "run_current_mA": 500,  "hold_current_mA": 150, "microsteps": 2,  "homing_dir": 0, "delay_us": 100, "home_offset": 0, "gear_ratio": 20.0, "dir_invert": -1},
    {"name": "Wrist", "address": 3, "step_pin": 21, "dir_pin": 20, "limit_min": 5, "limit_max": 4, "run_current_mA": 500,  "hold_current_mA": 150, "microsteps": 2,  "homing_dir": 0, "delay_us": 100, "home_offset": 0, "gear_ratio": 1.0, "dir_invert": 1},
]

# Set up non-blocking USB reading
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
    
    # Enable StealthChop for cooler, quieter operation (uses MicroPlyer for smoothness)
    tmc.set_stealthchop_enable(True) 
    tmc.enable()
    
    motors.append({
        # --- Legacy keys required by homer.py ---
        "name": config['name'],
        "tmc": tmc,
        "config": config,
        "step_pin": step_p,
        "dir_pin": dir_p,
        "limit_min_pin": limit_min_p,
        "limit_max_pin": limit_max_p,
        "current_position": 0,
        "target_position": 0,
        
        # --- Flattened keys for the new ROS 2 High-Speed Loop ---
        "step_pin_obj": step_p,
        "dir_pin_obj": dir_p,
        "homing_dir_val": config['homing_dir'],
        "dir_invert": config['dir_invert'],
        "steps_remaining": 0,
        "step_delay_us": 0,
        "last_step_time": time.ticks_us()
    })

# --- HOMING ROUTINE ---
try:
    print("Homing in reverse order")
    homer.home_all(motors[::-1], master_override_button)
    print("\nWaiting for ROS 2 Serial Commands")
except KeyboardInterrupt:
    print("Homing aborted.")

# --- THE SLAVE LISTENING LOOP ---
serial_buffer = ""

try:
    while True:
        # 1. READ ALL AVAILABLE SERIAL DATA
        while poller.poll(0):
            char = sys.stdin.read(1)
            if char:
                serial_buffer += char
                
        # 2. PARSE THE LATEST COMPLETE PACKET
        if '>' in serial_buffer:
            end_idx = serial_buffer.rfind('>')
            start_idx = serial_buffer.rfind('<', 0, end_idx)
            
            if start_idx != -1:
                clean_str = serial_buffer[start_idx + 1 : end_idx]
                str_targets = clean_str.split(',')
                
                if len(str_targets) >= len(motors):
                    for i in range(len(motors)):
                        try:
                            # 1. Parse raw radians directly from ROS
                            target_rad = float(str_targets[i])
                            
                            # 2. Convert radians to absolute steps dynamically
                            m_config = motors[i]["config"]
                            steps = (target_rad / (2.0 * math.pi)) * 200 * m_config["microsteps"] * m_config["gear_ratio"]
                            
                            # 3. Assign absolute position target
                            motors[i]["target_position"] = int(steps)
                            
                            # DEBUG: Log received commands
                            if i == 0:  # Only log base motor to avoid spam
                                deg = target_rad * 57.2958  # rad to degrees
                                out_deg = (steps / (200 * m_config["microsteps"] * m_config["gear_ratio"])) * 57.2958
                                print(f"[{m_config['name']}] RX: {target_rad:.4f}rad ({deg:.1f}°) -> {steps} steps (~{out_deg:.1f}°)")
                        except ValueError:
                            pass 
            
            serial_buffer = serial_buffer[end_idx + 1:]
            
        # 3. ABSOLUTE TRACKING STEPPING LOGIC
        now = time.ticks_us()
        steps_executed = 0
        
        for m in motors:
            # Calculate how far we are from MoveIt's commanded target
            distance_to_go = m["target_position"] - m["current_position"]
            
            if distance_to_go != 0:
                # A. Determine direction
                is_forward = (distance_to_go > 0)
                if m["dir_invert"] == -1:
                    is_forward = not is_forward
                    
                # B. Set direction pin
                if is_forward:
                    m["dir_pin_obj"].value(not m["homing_dir_val"])
                else:
                    m["dir_pin_obj"].value(m["homing_dir_val"])
                    
                # C. Dynamic P-Controller Speed logic
                # Proportional stepping: bigger error = faster catch-up
                # Min 5µs per step (200kHz) allows tracking of fast MoveIt trajectories
                steps_per_cycle = abs(distance_to_go) if abs(distance_to_go) > 100 else 100
                m["step_delay_us"] = max(MIN_STEP_DELAY_US, CONTROL_PERIOD_US // steps_per_cycle)
                
                # D. Execute Step
                if time.ticks_diff(now, m["last_step_time"]) >= m["step_delay_us"]:
                    
                    m["step_pin_obj"].high()
                    time.sleep_us(2)
                    m["step_pin_obj"].low()
                    
                    if distance_to_go > 0:
                        m["current_position"] += 1
                    else:
                        m["current_position"] -= 1
                    
                    m["last_step_time"] = now  # FIX: Update timestamp for next iteration
                    steps_executed += 1
        
        # DEBUG: Show stepping activity every 500 iterations (2.5 seconds at ~200Hz loop)
        if steps_executed > 0:
            print(f"Stepped: {steps_executed} motors")
                        
        m["last_step_time"] = now

except KeyboardInterrupt:
    print("\nProgram stopped.")
finally:
    for m in motors:
        m["tmc"].disable()
    print("Motors disabled.")