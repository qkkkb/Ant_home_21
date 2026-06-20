from machine import Pin
from seekfree import WIRELESS_UART
import gc
import utime

import config as cfg
from lsm6dsv16x_gyro_runtime import LSM6DSV16XYawRuntime


GYRO_SIGN = 1.0
GYRO_OFFSET_Z = 0.0
GYRO_SCALE = -1.0
GYRO_DEADBAND_DPS = 0.8
GYRO_CALIBRATE_SAMPLES = 1000
GYRO_CALIBRATE_DELAY_MS = 2
SAMPLE_PERIOD_MS = 5
STATIC_MS = 10000


wireless = None


def log(msg):
    print(msg)
    if wireless is not None:
        try:
            wireless.write(msg)
            wireless.write("\r\n")
        except Exception:
            pass


def yaw_delta(start_deg, end_deg):
    delta = end_deg - start_deg
    while delta > 180.0:
        delta -= 360.0
    while delta < -180.0:
        delta += 360.0
    return delta


def wait_key_release(key, exit_key):
    while key.value() == 0:
        if exit_key.value() == 0:
            raise KeyboardInterrupt
        utime.sleep_ms(10)


def wait_key_press(key, exit_key):
    wait_key_release(key, exit_key)
    while True:
        if exit_key.value() == 0:
            raise KeyboardInterrupt
        if key.value() == 0:
            utime.sleep_ms(20)
            if key.value() == 0:
                wait_key_release(key, exit_key)
                return
        utime.sleep_ms(10)


def sample_until_key(imu, key, exit_key, led):
    wait_key_release(key, exit_key)
    last_blink = utime.ticks_ms()
    while True:
        imu.read_gyro_z()
        now = utime.ticks_ms()
        if utime.ticks_diff(now, last_blink) >= 300:
            led.toggle()
            last_blink = now
        if exit_key.value() == 0:
            raise KeyboardInterrupt
        if key.value() == 0:
            utime.sleep_ms(20)
            if key.value() == 0:
                wait_key_release(key, exit_key)
                return
        utime.sleep_ms(SAMPLE_PERIOD_MS)


def sample_static(imu, led):
    start_yaw = imu.read_yaw()
    start_ms = utime.ticks_ms()
    last_blink = start_ms
    count = 0
    gyro_sum = 0.0
    gyro_abs_max = 0.0

    while utime.ticks_diff(utime.ticks_ms(), start_ms) < STATIC_MS:
        gyro_z = imu.read_gyro_z()
        gyro_sum += gyro_z
        abs_gyro = abs(gyro_z)
        if abs_gyro > gyro_abs_max:
            gyro_abs_max = abs_gyro
        count += 1
        now = utime.ticks_ms()
        if utime.ticks_diff(now, last_blink) >= 500:
            led.toggle()
            last_blink = now
        utime.sleep_ms(SAMPLE_PERIOD_MS)

    end_yaw = imu.read_yaw()
    avg_gyro = gyro_sum / count if count else 0.0
    return yaw_delta(start_yaw, end_yaw), avg_gyro, gyro_abs_max, count


def main():
    global wireless
    try:
        wireless = WIRELESS_UART(cfg.COOP_WIRELESS_BAUD)
    except Exception:
        wireless = None

    key_start = Pin(cfg.BTN_START_PIN, Pin.IN, Pin.PULL_UP)
    key_exit = Pin(cfg.BTN_EXIT_PIN, Pin.IN, Pin.PULL_UP)
    led = Pin(cfg.LED_HB_PIN, Pin.OUT, pull=Pin.PULL_UP_47K, value=True)
    led_straight = Pin(cfg.LED_STRAIGHT_PIN, Pin.OUT, value=0)
    led_translate = Pin(cfg.LED_TRANSLATE_PIN, Pin.OUT, value=0)
    led_rotate = Pin(cfg.LED_ROTATE_PIN, Pin.OUT, value=0)

    led_straight.value(1)
    led_translate.value(0)
    led_rotate.value(0)
    gc.collect()

    log("GYRO VERIFY")
    log("C9 START/OK C8 EXIT")
    log("KEEP STILL THEN C9")
    wait_key_press(key_start, key_exit)

    log("CAL START")
    imu = LSM6DSV16XYawRuntime(
        sign=GYRO_SIGN,
        offset_z=GYRO_OFFSET_Z,
        scale=GYRO_SCALE,
        deadband_dps=GYRO_DEADBAND_DPS,
        tick_period_ms=SAMPLE_PERIOD_MS,
    )
    imu.calibrate_offset(
        samples=GYRO_CALIBRATE_SAMPLES,
        delay_ms=GYRO_CALIBRATE_DELAY_MS,
        logger=log,
    )
    imu.reset_yaw(0.0)
    gc.collect()

    led_straight.value(0)
    led_translate.value(1)
    led_rotate.value(0)
    log("TURN 90 DEG THEN C9")
    yaw_start = imu.read_yaw()
    sample_until_key(imu, key_start, key_exit, led)
    yaw_end = imu.read_yaw()
    turn_delta = yaw_delta(yaw_start, yaw_end)
    log("TURN_DYAW %.2f" % turn_delta)
    log("TURN_ABS %.2f" % abs(turn_delta))

    led_straight.value(0)
    led_translate.value(0)
    led_rotate.value(1)
    log("STATIC 10S")
    drift, avg_gyro, max_gyro, count = sample_static(imu, led)
    log("DRIFT_10S %.2f" % drift)
    log("GYRO_AVG %.3f" % avg_gyro)
    log("GYRO_MAX %.3f" % max_gyro)
    log("N %d" % count)
    log("DONE")

    led.value(True)
    led_straight.value(0)
    led_translate.value(0)
    led_rotate.value(0)


try:
    main()
except KeyboardInterrupt:
    log("EXIT")
finally:
    try:
        Pin(cfg.LED_STRAIGHT_PIN, Pin.OUT, value=0)
        Pin(cfg.LED_TRANSLATE_PIN, Pin.OUT, value=0)
        Pin(cfg.LED_ROTATE_PIN, Pin.OUT, value=0)
    except Exception:
        pass
