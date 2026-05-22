H1 = 0xA5
H2 = 0x5A
MAX_LEN = 16

MSG_TARGET_LOCK = 0x10
MSG_ACK = 0x11
MSG_MASTER_READY = 0x12
MSG_SLAVE_APPROACHING = 0x13
MSG_SLAVE_READY = 0x14
MSG_PUSH_START = 0x15
MSG_PUSH_STOP = 0x16
MSG_DONE = 0x17
MSG_ERROR = 0x18
MSG_MASTER_MOTION = 0x19

ACK_TARGET_LOCK = MSG_TARGET_LOCK
ACK_PUSH_START = MSG_PUSH_START
ACK_PUSH_STOP = MSG_PUSH_STOP
MASTER_MOTION_FLAG_STARTED = 0x01
MASTER_MOTION_FLAG_TARGET = 0x02


class CoopFrameParser:
    def __init__(self):
        self.state = 0
        self.n = 0
        self.idx = 0
        self.sum = 0
        self.msg = 0
        self.seq = 0
        self.payload = bytearray(MAX_LEN)

    def reset(self):
        self.state = 0
        self.n = 0
        self.idx = 0
        self.sum = 0

    def feed(self, data, data_len, handler):
        for i in range(data_len):
            b = int(data[i]) & 0xFF
            if self.state == 0:
                if b == H1:
                    self.state = 1
            elif self.state == 1:
                if b == H2:
                    self.state = 2
                elif b != H1:
                    self.state = 0
            elif self.state == 2:
                if b < 2 or b > MAX_LEN:
                    self.reset()
                else:
                    self.n = b
                    self.idx = 0
                    self.sum = b
                    self.state = 3
            elif self.state == 3:
                self.sum = (self.sum + b) & 0xFF
                if self.idx == 0:
                    self.msg = b
                elif self.idx == 1:
                    self.seq = b
                else:
                    self.payload[self.idx - 2] = b
                self.idx += 1
                if self.idx >= self.n:
                    self.state = 4
            else:
                if b == self.sum:
                    handler(self.msg, self.seq, self.payload, self.n - 2)
                self.reset()


def _u16(data, off):
    v = data[off] | (data[off + 1] << 8)
    return v - 65536 if v >= 32768 else v


def _p16(v):
    v = int(v)
    if v > 32767:
        v = 32767
    elif v < -32768:
        v = -32768
    if v < 0:
        v += 65536
    return v & 0xFF, (v >> 8) & 0xFF


def encode_frame(msg, seq, payload=b""):
    n = 2 + len(payload)
    frame = bytearray([H1, H2, n, msg & 0xFF, seq & 0xFF])
    frame += payload
    s = 0
    for v in frame[2:]:
        s = (s + v) & 0xFF
    frame.append(s)
    return frame


def decode_target_lock(payload, payload_len):
    if payload_len < 10:
        return None
    return (
        payload[0],
        payload[1],
        _u16(payload, 2) / 10.0,
        _u16(payload, 4),
        _u16(payload, 6),
        _u16(payload, 8) / 10.0,
    )


def encode_master_motion(seq, vx, vy, wz, yaw_deg, flags):
    payload = bytearray()
    for v in (int(vx * 10), int(vy * 10), int(wz * 10), int(yaw_deg * 10)):
        lo, hi = _p16(v)
        payload.append(lo)
        payload.append(hi)
    payload.append(flags & 0xFF)
    return encode_frame(MSG_MASTER_MOTION, seq, payload)


def decode_master_motion(payload, payload_len):
    if payload_len < 9:
        return None
    return (
        _u16(payload, 0) / 10.0,
        _u16(payload, 2) / 10.0,
        _u16(payload, 4) / 10.0,
        _u16(payload, 6) / 10.0,
        payload[8],
    )
