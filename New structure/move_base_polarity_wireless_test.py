from array import array
from machine import Pin
from seekfree import WIRELESS_UART
import gc
import utime

import config as cfg
from hardware import Motor
from models import MoveBase
from move_base import calc_wheel_spd


RX_BUF_LEN = 48
LINE_BUF_LEN = 64
DEFAULT_BODY_SPEED = 3.0
DEFAULT_ROTATE_SPEED = 2.0
DEFAULT_RUN_MS = 600
PWM_PER_SPEED = 1600
LOOP_SLEEP_MS = 5
STATUS_PERIOD_MS = 1000
GC_DIV = 80

MODE_F = 0
MODE_B = 1
MODE_R = 2
MODE_L = 3
MODE_CW = 4
MODE_CCW = 5
MODE_COUNT = 6

MODE_NAMES = ("F", "B", "R", "L", "CW", "CCW")


def clamp_duty(value):
    value = int(value)
    if value > cfg.MOTOR_DUTY_MAX:
        return cfg.MOTOR_DUTY_MAX
    if value < -cfg.MOTOR_DUTY_MAX:
        return -cfg.MOTOR_DUTY_MAX
    return value


def apply_min_duty(value):
    value = clamp_duty(value)
    if value == 0:
        return 0
    if 0 < value < cfg.MOTOR_DUTY_MIN:
        return cfg.MOTOR_DUTY_MIN
    if -cfg.MOTOR_DUTY_MIN < value < 0:
        return -cfg.MOTOR_DUTY_MIN
    return value


def parse_float(parts, index, default):
    if len(parts) <= index:
        return default
    try:
        return float(parts[index])
    except Exception:
        return default


def parse_int(parts, index, default):
    if len(parts) <= index:
        return default
    try:
        return int(float(parts[index]))
    except Exception:
        return default


def mode_motion(mode):
    if mode == MODE_F:
        return DEFAULT_BODY_SPEED, 0.0, 0.0, DEFAULT_RUN_MS
    if mode == MODE_B:
        return -DEFAULT_BODY_SPEED, 0.0, 0.0, DEFAULT_RUN_MS
    if mode == MODE_R:
        return 0.0, DEFAULT_BODY_SPEED, 0.0, DEFAULT_RUN_MS
    if mode == MODE_L:
        return 0.0, -DEFAULT_BODY_SPEED, 0.0, DEFAULT_RUN_MS
    if mode == MODE_CW:
        return 0.0, 0.0, DEFAULT_ROTATE_SPEED, DEFAULT_RUN_MS
    return 0.0, 0.0, -DEFAULT_ROTATE_SPEED, DEFAULT_RUN_MS


class MotorRig:
    def __init__(self):
        self.fl = Motor(
            cfg.MOTOR_FL_PH,
            cfg.MOTOR_FL_PWM,
            freq=cfg.MOTOR_FREQ,
            invert=cfg.MOTOR_FL_INVERT,
        )
        self.fr = Motor(
            cfg.MOTOR_FR_PH,
            cfg.MOTOR_FR_PWM,
            freq=cfg.MOTOR_FREQ,
            invert=cfg.MOTOR_FR_INVERT,
        )
        self.b = Motor(
            cfg.MOTOR_B_PH,
            cfg.MOTOR_B_PWM,
            freq=cfg.MOTOR_FREQ,
            invert=cfg.MOTOR_B_INVERT,
        )

    def set_pwm(self, pwm_fl, pwm_fr, pwm_b):
        self.fl.duty(apply_min_duty(pwm_fl))
        self.fr.duty(apply_min_duty(pwm_fr))
        self.b.duty(apply_min_duty(pwm_b))

    def stop(self):
        self.fl.duty(0)
        self.fr.duty(0)
        self.b.duty(0)


class WirelessLineReader:
    def __init__(self):
        self.rx_buf = array("b", [0] * RX_BUF_LEN)
        self.line_buf = bytearray(LINE_BUF_LEN)
        self.line_len = 0

    def poll(self, wireless):
        try:
            n = wireless.receive_bytearray(self.rx_buf, RX_BUF_LEN)
        except Exception:
            return None
        if not n:
            return None
        i = 0
        while i < n:
            ch = int(self.rx_buf[i]) & 0xFF
            i += 1
            if ch == 10 or ch == 13:
                if self.line_len:
                    text = bytes(self.line_buf[: self.line_len]).decode()
                    self.line_len = 0
                    return text.strip()
            elif 32 <= ch <= 126:
                if self.line_len < LINE_BUF_LEN:
                    self.line_buf[self.line_len] = ch
                    self.line_len += 1
                else:
                    self.line_len = 0
        return None


class MoveBasePolarityTest:
    def __init__(self):
        gc.collect()
        self.wireless = WIRELESS_UART(cfg.COOP_WIRELESS_BAUD)
        self.reader = WirelessLineReader()
        self.motors = MotorRig()
        self.move = MoveBase()
        self.led = Pin(cfg.LED_HB_PIN, Pin.OUT, pull=Pin.PULL_UP_47K, value=True)
        self.led_straight = Pin(cfg.LED_STRAIGHT_PIN, Pin.OUT, value=0)
        self.led_translate = Pin(cfg.LED_TRANSLATE_PIN, Pin.OUT, value=0)
        self.led_rotate = Pin(cfg.LED_ROTATE_PIN, Pin.OUT, value=0)
        self.key_exit = Pin(cfg.BTN_EXIT_PIN, Pin.IN, Pin.PULL_UP)
        self.key_start = Pin(cfg.BTN_START_PIN, Pin.IN, Pin.PULL_UP)
        self.key_mode = Pin(cfg.BTN_MODE_PIN, Pin.IN, Pin.PULL_UP)
        self.last_exit_state = 1
        self.last_start_state = 1
        self.last_mode_state = 1
        self.mode = MODE_F
        self.stop_at_ms = 0
        self.active = False
        self.exit_requested = False
        self.last_status_ms = utime.ticks_ms()
        self.loop_count = 0
        self.update_mode_leds()

    def log(self, msg):
        print(msg)
        try:
            self.wireless.send_str(msg + "\r\n")
        except Exception:
            pass

    def help(self):
        self.log("keys: C14=next mode, C9=run mode, C8=stop/exit")
        self.log("mode order: F B R L CW CCW")
        self.log("wire: NEXT, RUN, STOP, HELP")
        self.log("wire: F/B/L/R [spd] [ms], CW/CCW [spd] [ms]")
        self.log("wire: MOVE vx vy vz [ms], M FL|FR|B duty [ms]")

    def update_mode_leds(self):
        straight = 1 if self.mode == MODE_F or self.mode == MODE_B else 0
        translate = 1 if self.mode == MODE_R or self.mode == MODE_L else 0
        rotate = 1 if self.mode == MODE_CW or self.mode == MODE_CCW else 0
        self.led_straight.value(straight)
        self.led_translate.value(translate)
        self.led_rotate.value(rotate)

    def log_mode(self):
        vx, vy, vz, run_ms = mode_motion(self.mode)
        self.log(
            "MODE %s body=%.2f,%.2f,%.2f ms=%d"
            % (MODE_NAMES[self.mode], vx, vy, vz, run_ms)
        )

    def select_next_mode(self):
        self.stop()
        self.mode += 1
        if self.mode >= MODE_COUNT:
            self.mode = 0
        self.update_mode_leds()
        self.log_mode()

    def stop(self):
        self.motors.stop()
        self.active = False
        self.stop_at_ms = 0

    def run_selected_mode(self):
        vx, vy, vz, run_ms = mode_motion(self.mode)
        self.apply_move(MODE_NAMES[self.mode], vx, vy, vz, run_ms)

    def apply_move(self, name, vx, vy, vz, run_ms):
        calc_wheel_spd(self.move, vx, vy, vz)
        pwm_fl = apply_min_duty(self.move.speed_fl * PWM_PER_SPEED)
        pwm_fr = apply_min_duty(self.move.speed_fr * PWM_PER_SPEED)
        pwm_b = apply_min_duty(self.move.speed_b * PWM_PER_SPEED)
        self.motors.set_pwm(pwm_fl, pwm_fr, pwm_b)
        self.stop_at_ms = utime.ticks_add(utime.ticks_ms(), int(run_ms))
        self.active = True
        self.log(
            "%s body=%.2f,%.2f,%.2f tar=%.2f,%.2f,%.2f pwm=%d,%d,%d ms=%d"
            % (
                name,
                vx,
                vy,
                vz,
                self.move.speed_fl,
                self.move.speed_fr,
                self.move.speed_b,
                pwm_fl,
                pwm_fr,
                pwm_b,
                int(run_ms),
            )
        )

    def apply_single_motor(self, role, duty, run_ms):
        role = role.upper()
        pwm_fl = 0
        pwm_fr = 0
        pwm_b = 0
        if role == "FL":
            pwm_fl = duty
        elif role == "FR":
            pwm_fr = duty
        elif role == "B":
            pwm_b = duty
        else:
            self.log("ERR bad motor role")
            return
        self.motors.set_pwm(pwm_fl, pwm_fr, pwm_b)
        self.stop_at_ms = utime.ticks_add(utime.ticks_ms(), int(run_ms))
        self.active = True
        self.log(
            "M %s pwm=%d,%d,%d ms=%d"
            % (
                role,
                apply_min_duty(pwm_fl),
                apply_min_duty(pwm_fr),
                apply_min_duty(pwm_b),
                int(run_ms),
            )
        )

    def check_exit_key(self):
        current = self.key_exit.value()
        if current == 0 and self.last_exit_state == 1:
            utime.sleep_ms(10)
            if self.key_exit.value() == 0:
                self.stop()
                self.exit_requested = True
                self.log("C8 EXIT")
        self.last_exit_state = current

    def check_start_key(self):
        current = self.key_start.value()
        if current == 0 and self.last_start_state == 1:
            utime.sleep_ms(10)
            if self.key_start.value() == 0:
                self.run_selected_mode()
        self.last_start_state = current

    def check_mode_key(self):
        current = self.key_mode.value()
        if current == 0 and self.last_mode_state == 1:
            utime.sleep_ms(10)
            if self.key_mode.value() == 0:
                self.select_next_mode()
        self.last_mode_state = current

    def handle_line(self, line):
        if not line:
            return
        parts = line.split()
        if not parts:
            return
        cmd = parts[0].upper()
        if cmd == "STOP" or cmd == "S":
            self.stop()
            self.log("STOP")
            return
        if cmd == "HELP" or cmd == "?":
            self.help()
            return
        if cmd == "NEXT" or cmd == "N":
            self.select_next_mode()
            return
        if cmd == "RUN" or cmd == "GO":
            self.run_selected_mode()
            return
        if cmd == "MODE":
            new_mode = parse_int(parts, 1, self.mode)
            if 0 <= new_mode < MODE_COUNT:
                self.stop()
                self.mode = new_mode
                self.update_mode_leds()
                self.log_mode()
            else:
                self.log("ERR mode range 0..5")
            return
        if cmd == "M":
            duty = parse_int(parts, 2, 1200)
            run_ms = parse_int(parts, 3, DEFAULT_RUN_MS)
            self.apply_single_motor(parts[1] if len(parts) > 1 else "", duty, run_ms)
            return
        if cmd == "MOVE":
            vx = parse_float(parts, 1, 0.0)
            vy = parse_float(parts, 2, 0.0)
            vz = parse_float(parts, 3, 0.0)
            run_ms = parse_int(parts, 4, DEFAULT_RUN_MS)
            self.apply_move("MOVE", vx, vy, vz, run_ms)
            return

        speed = parse_float(parts, 1, DEFAULT_BODY_SPEED)
        run_ms = parse_int(parts, 2, DEFAULT_RUN_MS)
        if cmd == "F":
            self.apply_move("F", speed, 0.0, 0.0, run_ms)
        elif cmd == "B":
            self.apply_move("B", -speed, 0.0, 0.0, run_ms)
        elif cmd == "L":
            self.apply_move("L", 0.0, -speed, 0.0, run_ms)
        elif cmd == "R":
            self.apply_move("R", 0.0, speed, 0.0, run_ms)
        elif cmd == "CW":
            self.apply_move("CW", 0.0, 0.0, speed, run_ms)
        elif cmd == "CCW":
            self.apply_move("CCW", 0.0, 0.0, -speed, run_ms)
        else:
            self.log("ERR unknown cmd")

    def run(self):
        self.log("=== move_base polarity key test ===")
        self.log(
            "motor inv fl=%s fr=%s b=%s pwm_per_speed=%d"
            % (cfg.MOTOR_FL_INVERT, cfg.MOTOR_FR_INVERT, cfg.MOTOR_B_INVERT, PWM_PER_SPEED)
        )
        self.help()
        self.log_mode()
        try:
            while not self.exit_requested:
                self.loop_count += 1
                now = utime.ticks_ms()
                self.check_exit_key()
                if self.exit_requested:
                    break
                self.check_mode_key()
                self.check_start_key()

                line = self.reader.poll(self.wireless)
                if line is not None:
                    self.handle_line(line)

                if self.active and utime.ticks_diff(now, self.stop_at_ms) >= 0:
                    self.stop()
                    self.log("AUTO STOP")

                if utime.ticks_diff(now, self.last_status_ms) >= STATUS_PERIOD_MS:
                    self.led.toggle()
                    self.last_status_ms = now
                    self.log(
                        "READY mode=%s active=%d"
                        % (MODE_NAMES[self.mode], 1 if self.active else 0)
                    )

                if self.loop_count % GC_DIV == 0:
                    gc.collect()

                utime.sleep_ms(LOOP_SLEEP_MS)
        finally:
            self.stop()
            self.led.value(True)
            self.led_straight.value(0)
            self.led_translate.value(0)
            self.led_rotate.value(0)
            self.log("=== stopped ===")


MoveBasePolarityTest().run()
