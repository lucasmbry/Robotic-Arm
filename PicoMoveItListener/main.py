from machine import Pin, UART, PWM
import time
import sys
import select
import math
import micropython
import _thread
import rp2
from tmc2209 import TMC2209

print("Waiting 3 seconds")
time.sleep(3)
micropython.kbd_intr(-1)

# ---------------------------------------------------------
# PACKET FORMAT (from host over USB serial):
#   <j1,j2,j3,j4,j5,j6>            joint targets in radians
#   <j1,j2,j3,j4,j5,j6,servo_deg>  optional 7th value = servo angle (0-180 deg)
# ---------------------------------------------------------

DEBUG = False            # Set True for verbose prints (slows the loops down)
LIMITS_ENABLED = True    # Block motion into a triggered hard limit switch

led = Pin("LED", Pin.OUT)

# Hardware setup for 2 UART buses
UART0_TX_PIN = 0
UART0_RX_PIN = 1
UART1_TX_PIN = 4
UART1_RX_PIN = 5
UART_BAUDRATE = 19200

# ---------------------------------------------------------
# SHIFT REGISTER SETUP (DIR pins, 74HC595)
# Only touched from Core 1 once the stepping thread is running.
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

def set_motor_direction(motor_index, dir_val):
    """Set a DIR bit; only clocks the register out if the bit actually changed."""
    global global_dir_state
    current = (global_dir_state >> motor_index) & 1
    if current == dir_val:
        return
    if dir_val:
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

set_servo_angle(90)

# ---------------------------------------------------------
# MOTOR CONFIGURATIONS
# ---------------------------------------------------------
MOTOR_CONFIGS = [
    {"name": "1st Axis", "uart_bus": 0, "address": 0, "shift_idx": 0, "step_pin": 16, "limit_min": 6,  "limit_max": 7,  "run_current_mA": 800,  "hold_current_mA": 300, "microsteps": 8, "homing_dir": 0, "gear_ratio": 15.0, "dir_invert": 1,  "max_speed_sps": 2000, "max_accel_sps2": 8000},
    {"name": "2nd Axis", "uart_bus": 0, "address": 1, "shift_idx": 1, "step_pin": 17, "limit_min": 8,  "limit_max": 9,  "run_current_mA": 1500, "hold_current_mA": 500, "microsteps": 8, "homing_dir": 0, "gear_ratio": 40.0, "dir_invert": 1,  "max_speed_sps": 5000, "max_accel_sps2": 8000},
    {"name": "3rd Axis", "uart_bus": 0, "address": 2, "shift_idx": 2, "step_pin": 18, "limit_min": 10, "limit_max": 11, "run_current_mA": 800,  "hold_current_mA": 300, "microsteps": 8, "homing_dir": 0, "gear_ratio": 20.0, "dir_invert": -1, "max_speed_sps": 3000, "max_accel_sps2": 8000},
    {"name": "4th Axis", "uart_bus": 0, "address": 3, "shift_idx": 3, "step_pin": 19, "limit_min": 12, "limit_max": 13, "run_current_mA": 800,  "hold_current_mA": 300, "microsteps": 8, "homing_dir": 0, "gear_ratio": 5.0,  "dir_invert": 1,  "max_speed_sps": 5000, "max_accel_sps2": 8000},
    {"name": "5th Axis", "uart_bus": 1, "address": 0, "shift_idx": 4, "step_pin": 20, "limit_min": 14, "limit_max": 15, "run_current_mA": 800,  "hold_current_mA": 300, "microsteps": 8, "homing_dir": 0, "gear_ratio": 3.0,  "dir_invert": -1, "max_speed_sps": 3000, "max_accel_sps2": 8000},
    {"name": "6th Axis", "uart_bus": 1, "address": 1, "shift_idx": 5, "step_pin": 21, "limit_min": 3,  "limit_max": 2,  "run_current_mA": 800,  "hold_current_mA": 300, "microsteps": 8, "homing_dir": 0, "gear_ratio": 1.0,  "dir_invert": 1,  "max_speed_sps": 5000, "max_accel_sps2": 8000},
]

poller = select.poll()
poller.register(sys.stdin, select.POLLIN)

uart0 = UART(0, baudrate=UART_BAUDRATE, tx=Pin(UART0_TX_PIN), rx=Pin(UART0_RX_PIN))
uart1 = UART(1, baudrate=UART_BAUDRATE, tx=Pin(UART1_TX_PIN), rx=Pin(UART1_RX_PIN))
uarts = [uart0, uart1]
motors = []

print("Drivers starting")
update_directions()  # Push initial zero state to shift register

mres_table = {0: 256, 1: 128, 2: 64, 3: 32, 4: 16, 5: 8, 6: 4, 7: 2, 8: 1}

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

    # Diagnostic
    ifcnt = tmc.read_int(0x02)
    gconf_after = tmc.read_int(tmc.REG_GCONF)
    chopconf = tmc.read_int(0x6C)
    mres = (chopconf >> 24) & 0x0F
    actual_microsteps = mres_table.get(mres, "UNKNOWN")
    print(f"[DIAG] {config['name']:>8} bus={config['uart_bus']} addr={config['address']} "
          f"IFCNT={ifcnt} mstep_reg_select={bool(gconf_after & (1 << 7))} "
          f"actual_microsteps={actual_microsteps}")

    motors.append({
        "config": config,
        "tmc": tmc,
        "step_pin_obj": step_p,
        "limit_min_pin": limit_min_p,
        "limit_max_pin": limit_max_p,
        "shift_idx": config['shift_idx'],
        "current_position": 0,
        "target_position": 0,
        "homing_dir_val": config['homing_dir'],
        "dir_invert": config['dir_invert'],
        "max_speed": config['max_speed_sps'],
        "max_accel": config['max_accel_sps2'],
        "velocity": 0.0,
        "step_accum": 0.0,
        "limit_tripped": False,
    })

position_lock = _thread.allocate_lock()

# PIO step generator
@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
def pio_stepper():
    wrap_target()
    pull(block)        # Pull word 1: delay_cycles
    mov(isr, osr)      # Save delay_cycles into ISR
    pull(block)        # Pull word 2: step_count
    mov(y, osr)        # Save step_count into Y

    jmp(y_dec, "step_loop")    # Decrement Y by 1 to align with N steps

    label("step_loop")
    set(pins, 1) [1]   # Drive Step HIGH (2us)
    set(pins, 0)       # Drive Step LOW

    mov(x, isr)        # Reload delay_cycles
    label("delay_loop")
    jmp(x_dec, "delay_loop")

    jmp(y_dec, "step_loop")
    wrap()

# Core 1 PIO motor stepping thread with trapezoidal velocity ramping
def motor_stepping_thread(lock, motor_array):

    print("CORE 1 on (PIO)")

    # 6 state machines: SM 0-3 live on PIO0, SM 4-5 on PIO1
    sms = []
    for i, m in enumerate(motor_array):
        sm = rp2.StateMachine(i, pio_stepper, freq=1_000_000, set_base=m["step_pin_obj"])
        sm.active(1)
        sms.append(sm)

    EVALUATION_INTERVAL_US = 5000
    last_loop_time = time.ticks_us()

    debug_tick = 0

    while True:
        try:
            now = time.ticks_us()
            dt_us = time.ticks_diff(now, last_loop_time)
            if dt_us <= 0: dt_us = 1
            dt_s = dt_us / 1_000_000.0
            last_loop_time = now

            debug_tick += 1
            do_print = DEBUG and (debug_tick % 100 == 0)

            if do_print:
                print(f"\n[CORE 1] --- Loop Active | dt_s: {dt_s:.5f} ---")

            for idx, m in enumerate(motor_array):
                lock.acquire()
                target = m["target_position"]
                current = m["current_position"]
                lock.release()

                dist = target - current
                velocity = m["velocity"]
                max_speed = m["max_speed"]
                max_accel = m["max_accel"]

                # ---- Hard limit protection ----
                if LIMITS_ENABLED and (dist != 0 or velocity != 0):
                    move_sign = velocity if velocity != 0 else dist
                    # Positive motion heads toward MIN only if homing_dir == 1
                    toward_min = (move_sign > 0) == (m["homing_dir_val"] == 1)
                    limit_pin = m["limit_min_pin"] if toward_min else m["limit_max_pin"]
                    if limit_pin.value() == 0:
                        if not m["limit_tripped"]:
                            print(f"[CORE 1] {m['config']['name']} hit {'MIN' if toward_min else 'MAX'} limit - stopping")
                            m["limit_tripped"] = True
                        lock.acquire()
                        m["target_position"] = m["current_position"]
                        lock.release()
                        m["velocity"] = 0.0
                        m["step_accum"] = 0.0
                        continue
                    else:
                        m["limit_tripped"] = False

                if dist == 0:
                    if velocity > 0:
                        velocity = max(0.0, velocity - max_accel * dt_s)
                    elif velocity < 0:
                        velocity = min(0.0, velocity + max_accel * dt_s)
                    if velocity == 0:
                        m["step_accum"] = 0.0
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

                    if dist != 0 and abs(velocity) < 15.0:
                        velocity = 15.0 if dist > 0 else -15.0

                m["velocity"] = velocity

                m["step_accum"] += velocity * dt_s
                steps_to_take = int(m["step_accum"])

                if do_print:
                    print(f"[CORE 1] {m['config']['name']:>8} | Cur: {current:<6} Tgt: {target:<6} Dist: {dist:<6} Vel: {velocity:.1f}")

                if steps_to_take != 0:
                    abs_steps = abs(steps_to_take)

                    if abs_steps > abs(dist):
                        abs_steps = abs(dist)
                        steps_to_take = abs_steps if steps_to_take > 0 else -abs_steps
                        clear_accum = True
                    else:
                        clear_accum = False

                    if abs_steps > 0:

                        if sms[idx].tx_fifo() < 3:

                            if clear_accum:
                                m["step_accum"] = 0.0
                            else:
                                m["step_accum"] -= steps_to_take

                            # Direction logic matches the working test script
                            is_forward = steps_to_take > 0
                            if m["dir_invert"] == -1:
                                is_forward = not is_forward

                            target_dir_val = 0 if m["homing_dir_val"] == 0 else 1
                            if is_forward:
                                target_dir_val = 1 if target_dir_val == 0 else 0

                            set_motor_direction(m["shift_idx"], target_dir_val)

                            delay_cycles = int((dt_s * 1_000_000) / abs_steps) - 5
                            delay_cycles = max(0, min(2147483647, delay_cycles))

                            if do_print:
                                print(f"[CORE 1] PUT idx={idx} steps={abs_steps} delay={delay_cycles}")

                            sms[idx].put(delay_cycles)
                            sms[idx].put(abs_steps)

                            lock.acquire()
                            m["current_position"] += steps_to_take
                            lock.release()

            elapsed = time.ticks_diff(time.ticks_us(), now)
            sleep_for = EVALUATION_INTERVAL_US - elapsed
            if sleep_for > 0:
                time.sleep_us(sleep_for)

        except Exception as e:
            print("[CORE 1] FATAL ERROR:")
            sys.print_exception(e)
            break


# Spawn the motor loop onto Core 1
_thread.start_new_thread(motor_stepping_thread, (position_lock, motors))

# Core 0 serial listener
serial_buffer = ""
packets_received = 0

print("Core 0 on (Serial Listener)")

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
                clean_str = serial_buffer[start_idx + 1: end_idx]

                if DEBUG:
                    print(f"\n[CORE 0] Packet {packets_received} Received: <{clean_str}>")

                str_targets = clean_str.split(',')

                if len(str_targets) >= len(motors):
                    for i in range(len(motors)):
                        try:
                            target_rad = float(str_targets[i])
                            m_config = motors[i]["config"]

                            steps = (target_rad / (2.0 * math.pi)) * 200 * m_config["microsteps"] * m_config["gear_ratio"]

                            if DEBUG:
                                print(f"[CORE 0] {m_config['name']:>9} | Rads: {target_rad:.5f} -> Target Steps: {int(steps)}")

                            position_lock.acquire()
                            motors[i]["target_position"] = int(steps)
                            position_lock.release()
                        except ValueError:
                            print(f"[CORE 0] ERROR: Failed to parse float for {motors[i]['config']['name']} from '{str_targets[i]}'")

                    # Optional 7th value: servo angle in degrees
                    if len(str_targets) > len(motors):
                        try:
                            set_servo_angle(float(str_targets[len(motors)]))
                        except ValueError:
                            print(f"[CORE 0] ERROR: Failed to parse servo angle from '{str_targets[len(motors)]}'")
                else:
                    print(f"[CORE 0] ERROR: Packet too short. Expected {len(motors)} targets, got {len(str_targets)}")

            serial_buffer = serial_buffer[end_idx + 1:]

        if len(serial_buffer) > 256:
            print("[CORE 0] WARNING: Serial buffer overflow, clearing buffer.")
            serial_buffer = ""

except KeyboardInterrupt:
    print("\n[CORE 0] KeyboardInterrupt detected. Shutting down.")
finally:
    position_lock.acquire()
    for m in motors:
        m["target_position"] = m["current_position"]
        m["tmc"].disable()
    position_lock.release()
    print("Motors disabled.")