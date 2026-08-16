import utime
import config as cfg
from models import MoveBase
from move_base import get_car_spd
from seekfree import WIRELESS_UART

_FRAME_LEN = 16
_TX_PERIOD_MS = cfg.MASTER_MOTION_TX_PERIOD_MS
_DIAG_ENABLE = cfg.MASTER_MOTION_DIAG_ENABLE
_DIAG_PERIOD_MS = cfg.MASTER_MOTION_DIAG_PERIOD_MS
_DIAG_BUF_SIZE = 68
_HEX = b"0123456789ABCDEF"
_PREVIEW_SCALE = 2.0
_STATE_PREVIEW = 0x80
_FLAG_STARTED = 0x01
_FLAG_TARGET = 0x02
_FLAG_CLOSED_LOOP = 0x04
_FLAG_ORBIT = 0x08
_FLAG_SPIN = 0x10
_FLAG_PUSH = 0x20
_FLAG_BACK = 0x40
_FLAG_RETURN = 0x80
_FILTER = 0.82
_LINEAR_DEADBAND = 0.5
_WZ_DEADBAND = 0.8

_wireless = None
_tx_buf = None
_diag_buf = None
_seq = 0
_last_tx_ms = 0
_last_diag_ms = 0
_last_state_code = -1
_motion_est = None
_vx = 0.0
_vy = 0.0
_wz = 0.0
_wheel_fl = 0.0
_wheel_fr = 0.0
_wheel_b = 0.0


def init():
    global _wireless, _tx_buf, _diag_buf, _motion_est
    _tx_buf = bytearray(16)
    _diag_buf = bytearray(_DIAG_BUF_SIZE) if _DIAG_ENABLE else None
    _motion_est = MoveBase()
    try:
        _wireless = WIRELESS_UART(cfg.COOP_WIRELESS_BAUD)
    except Exception:
        _wireless = None


def _deadband(value, deadband):
    if -deadband < value < deadband:
        return 0.0
    return value


def update_state(e_fl, e_fr, e_b, gyro_z):
    global _vx, _vy, _wz, _wheel_fl, _wheel_fr, _wheel_b
    if _motion_est is None:
        return
    get_car_spd(_motion_est, e_fr, e_fl, e_b)
    vx = _deadband(_motion_est.speed_x, _LINEAR_DEADBAND)
    vy = _deadband(_motion_est.speed_y, _LINEAR_DEADBAND)
    wz = _deadband(gyro_z, _WZ_DEADBAND)
    _vx += (vx - _vx) * _FILTER
    _vy += (vy - _vy) * _FILTER
    _wz += (wz - _wz) * _FILTER
    _wheel_fl += (e_fl - _wheel_fl) * _FILTER
    _wheel_fr += (e_fr - _wheel_fr) * _FILTER
    _wheel_b += (e_b - _wheel_b) * _FILTER


def _next_seq():
    global _seq
    _seq = (_seq + 1) & 0xFF
    if _seq == 0:
        _seq = 1
    return _seq


def _put_u8(idx, value):
    _tx_buf[idx] = int(value) & 0xFF


def _put_i16(idx, value):
    value = int(value)
    if value > 32767:
        value = 32767
    elif value < -32768:
        value = -32768
    if value < 0:
        value += 65536
    _tx_buf[idx] = value & 0xFF
    _tx_buf[idx + 1] = (value >> 8) & 0xFF


def _put_i8(idx, value):
    value = int(value)
    if value > 127:
        value = 127
    elif value < -128:
        value = -128
    _tx_buf[idx] = value & 0xFF


def _diag_hex8(buf, pos, value):
    buf[pos] = _HEX[(value >> 4) & 15]
    buf[pos + 1] = _HEX[value & 15]
    return pos + 2


def _diag_hex16(buf, pos, value):
    value = int(value) & 0xFFFF
    pos = _diag_hex8(buf, pos, value >> 8)
    return _diag_hex8(buf, pos, value)


def _send_diag(now, cmd_vx, cmd_vy, cmd_wz):
    global _last_diag_ms
    if (
        _diag_buf is None
        or utime.ticks_diff(now, _last_diag_ms) < _DIAG_PERIOD_MS
    ):
        return
    _last_diag_ms = now
    try:
        buf = _diag_buf
        buf[0] = 77
        buf[1] = 32
        pos = 2
        pos = _diag_hex16(buf, pos, now)
        buf[pos] = 32
        pos += 1
        i = 0
        while i < _FRAME_LEN:
            pos = _diag_hex8(buf, pos, _tx_buf[i])
            i += 1
        buf[pos] = 32
        pos += 1
        pos = _diag_hex16(buf, pos, _vx * 10.0)
        pos = _diag_hex16(buf, pos, _vy * 10.0)
        pos = _diag_hex16(buf, pos, _wz * 10.0)
        buf[pos] = 32
        pos += 1
        pos = _diag_hex16(buf, pos, cmd_vx * 10.0)
        pos = _diag_hex16(buf, pos, cmd_vy * 10.0)
        pos = _diag_hex16(buf, pos, cmd_wz * 10.0)
        buf[pos] = 13
        pos += 1
        buf[pos] = 10
        _wireless.send_bytearray(buf, pos + 1)
    except Exception:
        pass


def send_if_due(now, car_started, state_code, target_seen, yaw_deg, cmd_vx, cmd_vy, cmd_wz):
    global _last_tx_ms, _last_state_code
    if _wireless is None:
        return
    if (
        (state_code != 8 or _last_state_code == 8)
        and utime.ticks_diff(now, _last_tx_ms) < _TX_PERIOD_MS
    ):
        return
    flags = 0
    vx = 0.0
    vy = 0.0
    wz = 0.0
    preview_vx = 0.0
    preview_vy = 0.0
    preview_frame = False
    hard_stop = state_code == 17
    if car_started and not hard_stop:
        flags = _FLAG_STARTED | _FLAG_CLOSED_LOOP
        vx = _vx
        vy = _vy
        wz = _wz
        if state_code == 1:
            # 普通发车转向传递受控角速度指令，避免实测瞬态直接冲击从车。
            wz = cmd_wz
        elif (
            state_code == 2
            or state_code == 3
            or state_code == 8
            or state_code == 12
            or state_code == 14
        ):
            # Measured translation is the rigid-motion base.  The compact
            # command delta gives the follower acceleration preview without
            # making it chase a speed the leader has not reached yet.
            preview_vx = cmd_vx - _vx
            preview_vy = cmd_vy - _vy
            preview_frame = True
            vx = _vx
            vy = _vy
            wz = _wz
        elif state_code == 6:
            # Push preparation is strict following: publish measured motion as
            # the rigid base and keep only half of the command gap as preview.
            preview_vx = cmd_vx - _vx
            preview_vy = cmd_vy - _vy
            preview_frame = True
            vx = _vx
            vy = _vy
            wz = cmd_wz
        elif state_code == 10:
            vy = cmd_vy
            vx = cmd_vx
            wz = cmd_wz
        elif state_code == 4:
            # Classification stops only the leader feedforward.  The state is
            # sent explicitly so the follower can keep converging by vision.
            vx = 0.0
            vy = 0.0
            wz = cmd_wz
        elif state_code == 16:
            flags |= _FLAG_ORBIT
            # Search spin publishes measured wheel speeds.  The follower
            # keeps its front-right wheel anchored to the leader front-left
            # wheel instead of copying the leader body twist.
            vx = _wheel_fl
            vy = _wheel_fr
            wz = _wheel_b
        elif state_code == 5:
            flags |= _FLAG_ORBIT
            # Normal orbit keeps the existing body-twist contract.
            vx = _vx
            vy = _vy
            wz = cmd_wz
        elif state_code == 7:
            flags |= _FLAG_PUSH
            preview_vx = cmd_vx - _vx
            preview_vy = cmd_vy - _vy
            preview_frame = True
            vx = _vx
            vy = _vy
            wz = cmd_wz
        elif state_code == 11:
            vx = cmd_vx
            vy = cmd_vy
            wz = cmd_wz
        elif state_code == 9:
            flags |= _FLAG_SPIN
            vx = cmd_vx
            vy = cmd_vy
            wz = cmd_wz
        elif state_code == 13:
            flags |= _FLAG_ORBIT
            vx = cmd_vx
            vy = cmd_vy
            wz = cmd_wz
        elif state_code == 15:
            vx = 0.0
            vy = 0.0
            wz = 0.0
        if state_code == 8:
            flags |= _FLAG_BACK
        if 11 <= state_code <= 14:
            flags |= _FLAG_RETURN
    if target_seen and not hard_stop:
        flags |= _FLAG_TARGET
    _put_u8(0, 0xA5)
    _put_u8(1, 0x5A)
    _put_u8(2, 12)
    _put_u8(3, 0x19)
    _put_u8(4, _next_seq())
    _put_i16(5, vx * 10)
    _put_i16(7, vy * 10)
    _put_i16(9, wz * 10)
    if preview_frame:
        _put_i8(11, preview_vx * _PREVIEW_SCALE)
        _put_i8(12, preview_vy * _PREVIEW_SCALE)
        state_code |= _STATE_PREVIEW
    elif state_code == 16:
        _put_i16(11, _wz * 10)
    elif state_code == 5:
        _put_i16(11, _wz * 10)
    else:
        _put_i16(11, yaw_deg * 10)
    _put_u8(13, flags)
    _put_u8(14, state_code)
    checksum = 0
    i = 2
    while i < 15:
        checksum = (checksum + _tx_buf[i]) & 0xFF
        i += 1
    _put_u8(15, checksum)
    try:
        _wireless.send_bytearray(_tx_buf, _FRAME_LEN)
        _last_tx_ms = now
        _last_state_code = state_code & 0x7F
    except Exception:
        return
    _send_diag(now, cmd_vx, cmd_vy, cmd_wz)
