COOP_MSG_READY = 0x31
COOP_MSG_ROTATE = 0x32
COOP_MSG_PUSH = 0x33
COOP_MSG_DONE = 0x34
COOP_MSG_ERROR = 0x35

FRAME_HEAD_1 = 0xA5
FRAME_HEAD_2 = 0x5A
MAX_FRAME_LEN = 32

MSG_TARGET_LOCK = 0x10
MSG_ACK = 0x11
MSG_MASTER_READY = 0x12
MSG_SLAVE_APPROACHING = 0x13
MSG_SLAVE_READY = 0x14
MSG_PUSH_START = 0x15
MSG_PUSH_STOP = 0x16
MSG_DONE = 0x17
MSG_ERROR = 0x18

ACK_TARGET_LOCK = MSG_TARGET_LOCK
ACK_PUSH_START = MSG_PUSH_START
ACK_PUSH_STOP = MSG_PUSH_STOP

TARGET_BEAR = 1
TARGET_SANDBAG = 2
TARGET_TENNIS = 3


class CoopProtocol:
    IDLE = 0
    SEARCHING = 1
    READY = 2
    ROTATING = 3
    PUSHING = 4
    DONE = 5
    ERROR = 6

    TARGET_LOCK = MSG_TARGET_LOCK
    ACK = MSG_ACK
    MASTER_READY = MSG_MASTER_READY
    SLAVE_APPROACHING = MSG_SLAVE_APPROACHING
    SLAVE_READY = MSG_SLAVE_READY
    PUSH_START = MSG_PUSH_START
    PUSH_STOP = MSG_PUSH_STOP
    DONE_MSG = MSG_DONE
    ERROR_MSG = MSG_ERROR


class CoopFrameParser:
    def __init__(self):
        self.buf = bytearray()

    def feed(self, data):
        frames = []
        if not data:
            return frames
        for b in data:
            self.buf.append(b & 0xFF)

        while len(self.buf) >= 6:
            if self.buf[0] != FRAME_HEAD_1 or self.buf[1] != FRAME_HEAD_2:
                self.buf = self.buf[1:]
                continue

            frame_len = int(self.buf[2])
            if frame_len < 2 or frame_len > MAX_FRAME_LEN:
                self.buf = self.buf[1:]
                continue

            total_len = frame_len + 4
            if len(self.buf) < total_len:
                break

            checksum = 0
            for value in self.buf[2:3 + frame_len]:
                checksum = (checksum + int(value)) & 0xFF
            if checksum != self.buf[3 + frame_len]:
                self.buf = self.buf[1:]
                continue

            msg_type = int(self.buf[3])
            seq = int(self.buf[4])
            payload = bytes(self.buf[5:3 + frame_len])
            frames.append((msg_type, seq, payload))
            self.buf = self.buf[total_len:]

        if len(self.buf) > 96:
            self.buf = self.buf[-4:]
        return frames


def _clamp_i16(value):
    value = int(value)
    if value > 32767:
        return 32767
    if value < -32768:
        return -32768
    return value


def _pack_i16(value):
    value = _clamp_i16(value)
    if value < 0:
        value += 65536
    return value & 0xFF, (value >> 8) & 0xFF


def _unpack_i16(data, offset):
    value = int(data[offset]) | (int(data[offset + 1]) << 8)
    if value >= 32768:
        value -= 65536
    return value


def encode_frame(msg_type, seq, payload=None):
    if payload is None:
        payload = b""
    frame_len = 2 + len(payload)
    frame = bytearray([FRAME_HEAD_1, FRAME_HEAD_2, frame_len & 0xFF, msg_type & 0xFF, seq & 0xFF])
    for b in payload:
        frame.append(b & 0xFF)
    checksum = 0
    for value in frame[2:]:
        checksum = (checksum + int(value)) & 0xFF
    frame.append(checksum)
    return bytes(frame)


def encode_ack(seq, ack_msg_type):
    return encode_frame(MSG_ACK, seq, bytes([ack_msg_type & 0xFF]))


def encode_target_lock(seq, master_dir, slave_dir, master_yaw_deg, err_x, err_y, push_speed):
    yaw10 = int(master_yaw_deg * 10.0)
    speed10 = int(push_speed * 10.0)
    payload = bytearray([master_dir & 0xFF, slave_dir & 0xFF])
    for value in (yaw10, err_x, err_y, speed10):
        lo, hi = _pack_i16(value)
        payload.append(lo)
        payload.append(hi)
    return encode_frame(MSG_TARGET_LOCK, seq, payload)


def decode_target_lock(payload):
    if len(payload) < 10:
        return None
    return {
        "master_dir": int(payload[0]),
        "slave_dir": int(payload[1]),
        "master_yaw_deg": _unpack_i16(payload, 2) / 10.0,
        "err_x": _unpack_i16(payload, 4),
        "err_y": _unpack_i16(payload, 6),
        "push_speed": _unpack_i16(payload, 8) / 10.0,
    }


def encode_status(msg_type, seq, nav_code=0, detail=0):
    return encode_frame(msg_type, seq, bytes([nav_code & 0xFF, detail & 0xFF]))


def decode_status(payload):
    nav_code = int(payload[0]) if len(payload) > 0 else 0
    detail = int(payload[1]) if len(payload) > 1 else 0
    return nav_code, detail


TARGET_PROFILE = {
    TARGET_BEAR: {
        "name": "bear",
        "rotate_deg": 90,
        "push_speed": 95.0,
        "align_tol": 14.0,
        "push_timeout": 300,
    },
    TARGET_SANDBAG: {
        "name": "sandbag",
        "rotate_deg": 80,
        "push_speed": 85.0,
        "align_tol": 12.0,
        "push_timeout": 260,
    },
    TARGET_TENNIS: {
        "name": "tennis",
        "rotate_deg": 60,
        "push_speed": 55.0,
        "align_tol": 10.0,
        "push_timeout": 180,
    },
}


def encode_partner_state(state, payload=0):
    return bytes([int(state) & 0xFF, int(payload) & 0xFF])


def decode_partner_packet(data):
    if not data:
        return None
    if len(data) == 1:
        return int(data[0]), 0
    return int(data[0]), int(data[1])


def get_target_profile(label):
    return TARGET_PROFILE.get(
        int(label),
        {
            "name": "unknown",
            "rotate_deg": 90,
            "push_speed": 80.0,
            "align_tol": 12.0,
            "push_timeout": 240,
        },
    )


def update_partner_flags(ctx, state, payload=0):
    ctx.partner_state = int(state)
    ctx.partner_payload = int(payload)
    ctx.partner_ready = 1 if state >= CoopProtocol.READY else 0
