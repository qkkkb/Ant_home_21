import utime
from seekfree import WIRELESS_UART


_PERIOD_MS = 100
_wireless = None
_last_ms = 0


def init(baud):
    global _wireless
    try:
        _wireless = WIRELESS_UART(baud)
    except Exception:
        _wireless = None


def send(now, state, yaw_ref, yaw, yaw_err, gyro_z, turn_cmd, vz_cmd, orbit_remain, pwm_fl, pwm_fr, pwm_b):
    global _last_ms
    if _wireless is None or utime.ticks_diff(now, _last_ms) < _PERIOD_MS:
        return
    _last_ms = now
    try:
        _wireless.send_str(
            "G %d %d %d %d %d %d %d %d %d %d %d\r\n"
            % (state, yaw_ref, yaw, yaw_err, gyro_z, turn_cmd, vz_cmd, orbit_remain, pwm_fl, pwm_fr, pwm_b)
        )
    except Exception:
        pass
