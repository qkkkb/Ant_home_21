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

ACK_TARGET_LOCK = MSG_TARGET_LOCK


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


def encode_target_lock(seq, master_dir, slave_dir, yaw_deg, err_x, err_y, push_speed):
    payload = bytearray([master_dir & 0xFF, slave_dir & 0xFF])
    for v in (int(yaw_deg * 10), err_x, err_y, int(push_speed * 10)):
        lo, hi = _p16(v)
        payload.append(lo)
        payload.append(hi)
    return encode_frame(MSG_TARGET_LOCK, seq, payload)
