BODY_X_SIGN = -1.0


def calc_wheel_spd(move, vx, vy, vz):
    vx = BODY_X_SIGN * vx
    move.speed_fr = vx * 0.866025 + vy * 0.5 + vz
    move.speed_fl = -vx * 0.866025 + vy * 0.5 + vz
    move.speed_b = -vy + vz


def calc_wheel_spd_into(out, vx, vy, vz):
    vx = BODY_X_SIGN * vx
    out[1] = vx * 0.866025 + vy * 0.5 + vz
    out[0] = -vx * 0.866025 + vy * 0.5 + vz
    out[2] = -vy + vz
