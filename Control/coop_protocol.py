COOP_MSG_READY = 0x31
COOP_MSG_ROTATE = 0x32
COOP_MSG_PUSH = 0x33
COOP_MSG_DONE = 0x34
COOP_MSG_ERROR = 0x35

TARGET_BEAR = 1
TARGET_SANDBAG = 2
TARGET_TENNIS = 3


class CoopProtocol:
    """
    两车协同协议建议：
    - 单字节状态位用于快速同步阶段
    - 可选第二字节作为 payload，传目标类别/错误码/阶段编号
    """

    IDLE = 0
    SEARCHING = 1
    READY = 2
    ROTATING = 3
    PUSHING = 4
    DONE = 5
    ERROR = 6


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
