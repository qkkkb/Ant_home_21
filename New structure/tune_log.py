import utime
from seekfree import WIRELESS_UART


_PERIOD_MS = 20
_BUF_SIZE = 160
_wireless = None
_buf = None
_last_ms = 0


def init(baud):
    global _wireless, _buf
    try:
        _wireless = WIRELESS_UART(baud)
        _buf = bytearray(_BUF_SIZE)
    except Exception:
        _wireless = None
        _buf = None


def _put_int(buf, pos, value):
    value = int(value)
    if value < 0:
        buf[pos] = 45
        pos += 1
        value = -value

    start = pos
    if value == 0:
        buf[pos] = 48
        pos += 1
    else:
        while value:
            buf[pos] = 48 + value % 10
            value //= 10
            pos += 1
        end = pos - 1
        while start < end:
            temp = buf[start]
            buf[start] = buf[end]
            buf[end] = temp
            start += 1
            end -= 1

    buf[pos] = 32
    return pos + 1


def send(
    now, state, yaw_ref, yaw, yaw_err, gyro_z, turn_cmd, vz_cmd,
    orbit_remain, encoder_dt_ms, target_fl, target_fr, target_b,
    enc_fl, enc_fr, enc_b, pwm_fl, pwm_fr, pwm_b,
):
    global _last_ms
    if (
        (state != 5 and state != 9 and state != 13 and state < 16)
        or _wireless is None
        or _buf is None
        or utime.ticks_diff(now, _last_ms) < _PERIOD_MS
    ):
        return
    _last_ms = now
    try:
        buf = _buf
        buf[0] = 71
        buf[1] = 32
        pos = 2
        pos = _put_int(buf, pos, state)
        pos = _put_int(buf, pos, yaw_ref)
        pos = _put_int(buf, pos, yaw)
        pos = _put_int(buf, pos, yaw_err)
        pos = _put_int(buf, pos, gyro_z)
        pos = _put_int(buf, pos, turn_cmd)
        pos = _put_int(buf, pos, vz_cmd)
        pos = _put_int(buf, pos, orbit_remain)
        pos = _put_int(buf, pos, encoder_dt_ms)
        pos = _put_int(buf, pos, target_fl)
        pos = _put_int(buf, pos, target_fr)
        pos = _put_int(buf, pos, target_b)
        pos = _put_int(buf, pos, enc_fl)
        pos = _put_int(buf, pos, enc_fr)
        pos = _put_int(buf, pos, enc_b)
        pos = _put_int(buf, pos, pwm_fl)
        pos = _put_int(buf, pos, pwm_fr)
        pos = _put_int(buf, pos, pwm_b)
        buf[pos - 1] = 13
        buf[pos] = 10
        _wireless.send_bytearray(buf, pos + 1)
    except Exception:
        pass
