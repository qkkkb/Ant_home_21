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
        self.buf = bytearray()

    def feed(self, data):
        out = []
        if not data:
            return out
        self.buf += data
        while len(self.buf) >= 6:
            if self.buf[0] != H1 or self.buf[1] != H2:
                self.buf = self.buf[1:]
                continue
            n = self.buf[2]
            if n < 2 or n > MAX_LEN:
                self.buf = self.buf[1:]
                continue
            total = n + 4
            if len(self.buf) < total:
                break
            s = 0
            for v in self.buf[2:3 + n]:
                s = (s + v) & 0xFF
            if s == self.buf[3 + n]:
                out.append((self.buf[3], self.buf[4], self.buf[5:3 + n]))
                self.buf = self.buf[total:]
            else:
                self.buf = self.buf[1:]
        if len(self.buf) > 32:
            self.buf = bytearray()
        return out


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


def encode_ack(seq, ack_msg):
    return encode_frame(MSG_ACK, seq, bytearray([ack_msg & 0xFF]))


def encode_status(msg, seq, nav=0, detail=0):
    return encode_frame(msg, seq, bytearray([nav & 0xFF, detail & 0xFF]))


def encode_target_lock(seq, master_dir, slave_dir, yaw_deg, err_x, err_y, push_speed):
    payload = bytearray([master_dir & 0xFF, slave_dir & 0xFF])
    for v in (int(yaw_deg * 10), err_x, err_y, int(push_speed * 10)):
        lo, hi = _p16(v)
        payload.append(lo)
        payload.append(hi)
    return encode_frame(MSG_TARGET_LOCK, seq, payload)
