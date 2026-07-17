BODY_X_SIGN = -1.0


def calc_wheel_spd(move, vx, vy, vz):
    vx = BODY_X_SIGN * vx
    move.speed_fr = vx * 0.866025 + vy * 0.5 + vz
    move.speed_fl = -vx * 0.866025 + vy * 0.5 + vz
    move.speed_b = -vy + vz


def calc_follow_wheel_spd(
    move, corr_vx, corr_vy, corr_vz, ff_vx, ff_vy, allow_reverse
):
    calc_wheel_spd(move, ff_vx, ff_vy, 0.0)
    ff_fl = move.speed_fl
    ff_fr = move.speed_fr
    ff_b = move.speed_b
    calc_wheel_spd(move, corr_vx, corr_vy, corr_vz)
    fl = ff_fl + move.speed_fl
    fr = ff_fr + move.speed_fr
    b = ff_b + move.speed_b
    if not allow_reverse:
        if ff_fl * fl < 0.0:
            fl = 0.0
        if ff_fr * fr < 0.0:
            fr = 0.0
        if ff_b * b < 0.0:
            b = 0.0
    target_max = max(abs(fl), abs(fr), abs(b))
    if target_max > 24.0:
        scale = 24.0 / target_max
        fl *= scale
        fr *= scale
        b *= scale
    move.speed_fl = fl
    move.speed_fr = fr
    move.speed_b = b
    return (move.speed_fl + move.speed_fr + move.speed_b) / 3.0
