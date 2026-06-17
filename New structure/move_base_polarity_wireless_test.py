from array import array
from machine import Pin
from seekfree import WIRELESS_UART
from smartcar import ticker, encoder
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
FEEDBACK_PERIOD_MS = 100
GC_DIV = 80

MODE_F = 0
MODE_B = 1
MODE_R = 2
MODE_L = 3
MODE_CW = 4
MODE_CCW = 5
MODE_COUNT = 6

MODE_NAMES = ("F", "B", "R", "L", "CW", "CCW")
MODE_LED_TABLE = (
    (1, 0, 0),
    (0, 1, 0),
    (0, 0, 1),
    (1, 1, 0),
    (1, 0, 1),
    (0, 1, 1),
)

_pit_flag = False


def time_pit_handler(_):
    global _pit_flag
    _pit_flag = True


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


def sign_char(value, deadband):
    if value > deadband:
        return "+"
    if value < -deadband:
        return "-"
    return "0"


class MotorRig:
    def __init__(self):
        self.last_pwm_fl = 0
        self.last_pwm_fr = 0
        self.last_pwm_b = 0
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
        self.last_pwm_fl = apply_min_duty(pwm_fl)
        self.last_pwm_fr = apply_min_duty(pwm_fr)
        self.last_pwm_b = apply_min_duty(pwm_b)
        self.fl.duty(self.last_pwm_fl)
        self.fr.duty(self.last_pwm_fr)
        self.b.duty(self.last_pwm_b)
        return self.last_pwm_fl, self.last_pwm_fr, self.last_pwm_b

    def stop(self):
        self.fl.duty(0)
        self.fr.duty(0)
        self.b.duty(0)
        self.last_pwm_fl = 0
        self.last_pwm_fr = 0
        self.last_pwm_b = 0


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
                    text = bytes(self.line_buf[: self.line_len]).decode("ascii", "ignore")
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
        self.enc_fl = encoder(cfg.ENC_FL_A, cfg.ENC_FL_B, cfg.ENC_FL_INVERT)
        self.enc_fr = encoder(cfg.ENC_FR_A, cfg.ENC_FR_B, cfg.ENC_FR_INVERT)
        self.enc_b = encoder(cfg.ENC_B_A, cfg.ENC_B_B, cfg.ENC_B_INVERT)
        self.pit = ticker(1)
        self.pit.capture_list(self.enc_fl, self.enc_fr, self.enc_b)
        self.pit.callback(time_pit_handler)
        self.pit.start(cfg.TICK_PERIOD_MS)
        self.led = Pin(cfg.LED_HB_PIN, Pin.OUT, pull=Pin.PULL_UP_47K, value=True)
        self.led_straight = Pin(cfg.LED_STRAIGHT_PIN, Pin.OUT, value=0)
        self.led_translate = Pin(cfg.LED_TRANSLATE_PIN, Pin.OUT, value=0)
        self.led_rotate = Pin(cfg.LED_ROTATE_PIN, Pin.OUT, value=0)
        self.key_exit = Pin(cfg.BTN_EXIT_PIN, Pin.IN, Pin.PULL_UP)
        self.key_start = Pin(cfg.BTN_START_PIN, Pin.IN, Pin.PULL_UP)
        self.key_mode = Pin(cfg.BTN_MODE_PIN, Pin.IN, Pin.PULL_UP)
        self.mode = MODE_F
        self.key_items = (
            (self.key_mode, self._on_mode_key, "MODE"),
            (self.key_start, self._on_start_key, "RUN"),
            (self.key_exit, self._on_exit_key, "EXIT"),
        )
        self.key_last = [1, 1, 1]
        self.stop_at_ms = 0
        self.active = False
        self.active_name = "IDLE"
        self.last_feedback_ms = 0
        self.last_tar_fl = 0.0
        self.last_tar_fr = 0.0
        self.last_tar_b = 0.0
        self.last_enc_fl = 0
        self.last_enc_fr = 0
        self.last_enc_b = 0
        self.exit_requested = False
        self.last_status_ms = utime.ticks_ms()
        self.loop_count = 0
        self.update_mode_leds()

    def log(self, msg):
        if not isinstance(msg, str):
            msg = str(msg)
        try:
            msg = msg.encode("ascii", "ignore").decode("ascii")
        except Exception:
            pass
        print(msg)
        try:
            self.wireless.send_str(msg + "\r\n")
        except Exception:
            pass

    def help(self):
        self.log("KEYS C14 NEXT C9 RUN C8 STOP")
        self.log("MODES F B R L CW CCW")
        self.log("WIRE NEXT RUN STOP HELP")
        self.log("WIRE F/B/L/R [SPD] [MS]")
        self.log("WIRE CW/CCW [SPD] [MS] MOVE VX VY VZ [MS]")
        self.log("WIRE M FL|FR|B DUTY [MS]")

    def update_mode_leds(self):
        straight, translate, rotate = MODE_LED_TABLE[self.mode]
        self.led_straight.value(straight)
        self.led_translate.value(translate)
        self.led_rotate.value(rotate)

    def log_mode(self):
        vx, vy, vz, run_ms = mode_motion(self.mode)
        leds = MODE_LED_TABLE[self.mode]
        self.log(
            "MODE %s LED=%d%d%d BODY=%.2f,%.2f,%.2f MS=%d"
            % (
                MODE_NAMES[self.mode],
                leds[0],
                leds[1],
                leds[2],
                vx,
                vy,
                vz,
                run_ms,
            )
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
        self.active_name = "IDLE"

    def run_selected_mode(self):
        vx, vy, vz, run_ms = mode_motion(self.mode)
        self.apply_move(MODE_NAMES[self.mode], vx, vy, vz, run_ms)

    def _on_mode_key(self):
        self.select_next_mode()

    def _on_start_key(self):
        self.run_selected_mode()

    def _on_exit_key(self):
        self.stop()
        self.exit_requested = True
        self.log("KEY EXIT")

    def poll_keys(self):
        for idx, item in enumerate(self.key_items):
            btn = item[0]
            cur = btn.value()
            if self.key_last[idx] == 1 and cur == 0:
                utime.sleep_ms(20)
                if btn.value() == 0:
                    item[1]()
                    if idx != 2:
                        self.log("KEY %s" % item[2])
            self.key_last[idx] = cur

    def read_encoders(self):
        self.last_enc_fl = int(self.enc_fl.get())
        self.last_enc_fr = int(self.enc_fr.get())
        self.last_enc_b = int(self.enc_b.get())
        return self.last_enc_fl, self.last_enc_fr, self.last_enc_b

    def log_feedback(self, prefix):
        e_fl, e_fr, e_b = self.read_encoders()
        pwm_fl = self.motors.last_pwm_fl
        pwm_fr = self.motors.last_pwm_fr
        pwm_b = self.motors.last_pwm_b
        self.log(
            "%s %s TAR=%.2f,%.2f,%.2f PWM=%d,%d,%d DIR=%s,%s,%s ENC=%d,%d,%d ESIGN=%s,%s,%s"
            % (
                prefix,
                self.active_name,
                self.last_tar_fl,
                self.last_tar_fr,
                self.last_tar_b,
                pwm_fl,
                pwm_fr,
                pwm_b,
                sign_char(pwm_fl, 0),
                sign_char(pwm_fr, 0),
                sign_char(pwm_b, 0),
                e_fl,
                e_fr,
                e_b,
                sign_char(e_fl, 1),
                sign_char(e_fr, 1),
                sign_char(e_b, 1),
            )
        )

    def finish_active(self, reason):
        if self.active:
            self.log_feedback(reason)
        self.stop()

    def apply_move(self, name, vx, vy, vz, run_ms):
        calc_wheel_spd(self.move, vx, vy, vz)
        self.last_tar_fl = self.move.speed_fl
        self.last_tar_fr = self.move.speed_fr
        self.last_tar_b = self.move.speed_b
        pwm_fl, pwm_fr, pwm_b = self.motors.set_pwm(
            self.last_tar_fl * PWM_PER_SPEED,
            self.last_tar_fr * PWM_PER_SPEED,
            self.last_tar_b * PWM_PER_SPEED,
        )
        self.stop_at_ms = utime.ticks_add(utime.ticks_ms(), int(run_ms))
        self.active = True
        self.active_name = name
        self.last_feedback_ms = utime.ticks_ms()
        self.log(
            "RUN %s BODY=%.2f,%.2f,%.2f TAR=%.2f,%.2f,%.2f PWM=%d,%d,%d DIR=%s,%s,%s MS=%d"
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
                sign_char(pwm_fl, 0),
                sign_char(pwm_fr, 0),
                sign_char(pwm_b, 0),
                int(run_ms),
            )
        )
        self.log_feedback("FB")

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
        pwm_fl, pwm_fr, pwm_b = self.motors.set_pwm(pwm_fl, pwm_fr, pwm_b)
        self.last_tar_fl = 0.0
        self.last_tar_fr = 0.0
        self.last_tar_b = 0.0
        self.stop_at_ms = utime.ticks_add(utime.ticks_ms(), int(run_ms))
        self.active = True
        self.active_name = "M" + role
        self.last_feedback_ms = utime.ticks_ms()
        self.log(
            "M %s PWM=%d,%d,%d DIR=%s,%s,%s MS=%d"
            % (
                role,
                pwm_fl,
                pwm_fr,
                pwm_b,
                sign_char(pwm_fl, 0),
                sign_char(pwm_fr, 0),
                sign_char(pwm_b, 0),
                int(run_ms),
            )
        )
        self.log_feedback("FB")

    def handle_line(self, line):
        if not line:
            return
        parts = line.split()
        if not parts:
            return
        cmd = parts[0].upper()
        if cmd == "STOP" or cmd == "S":
            self.finish_active("STOP")
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
                self.finish_active("STOP")
                self.mode = new_mode
                self.update_mode_leds()
                self.log_mode()
            else:
                self.log("ERR MODE 0..5")
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
            self.log("ERR CMD")

    def run(self):
        self.log("=== KEY TEST START ===")
        self.log(
            "MOTOR INV FL=%s FR=%s B=%s PPS=%d"
            % (cfg.MOTOR_FL_INVERT, cfg.MOTOR_FR_INVERT, cfg.MOTOR_B_INVERT, PWM_PER_SPEED)
        )
        self.help()
        self.log_mode()
        try:
            while not self.exit_requested:
                self.loop_count += 1
                now = utime.ticks_ms()
                self.poll_keys()
                if self.exit_requested:
                    break

                line = self.reader.poll(self.wireless)
                if line is not None:
                    self.handle_line(line)

                if self.active and utime.ticks_diff(now, self.stop_at_ms) >= 0:
                    self.finish_active("DONE")

                if self.active and utime.ticks_diff(now, self.last_feedback_ms) >= FEEDBACK_PERIOD_MS:
                    self.last_feedback_ms = now
                    self.log_feedback("FB")

                if utime.ticks_diff(now, self.last_status_ms) >= STATUS_PERIOD_MS:
                    self.led.toggle()
                    self.last_status_ms = now
                    self.log(
                        "READY MODE=%s ACTIVE=%d"
                        % (MODE_NAMES[self.mode], 1 if self.active else 0)
                    )

                if self.loop_count % GC_DIV == 0:
                    gc.collect()

                utime.sleep_ms(LOOP_SLEEP_MS)
        finally:
            try:
                self.pit.stop()
            except Exception:
                pass
            self.stop()
            self.led.value(True)
            self.led_straight.value(0)
            self.led_translate.value(0)
            self.led_rotate.value(0)
            self.log("=== stopped ===")


MoveBasePolarityTest().run()
