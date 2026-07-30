H1 = 0xA5
H2 = 0x5A
MAX_LEN = 16

MSG_MASTER_MOTION = 0x19

MASTER_MOTION_FLAG_STARTED = 0x01
MASTER_MOTION_FLAG_ORBIT = 0x08
MASTER_MOTION_FLAG_SPIN = 0x10
MASTER_MOTION_FLAG_PUSH = 0x20
MASTER_MOTION_FLAG_BACK = 0x40
MASTER_MOTION_FLAG_RETURN = 0x80


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


def decode_i16(data, off):
    v = data[off] | (data[off + 1] << 8)
    return v - 65536 if v >= 32768 else v
