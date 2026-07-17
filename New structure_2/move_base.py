BODY_X_SIGN = -1.0


def calc_wheel_spd(move, vx, vy, vz):
    vx = BODY_X_SIGN * vx
    move.speed_fr = vx * 0.866025 + vy * 0.5 + vz
    move.speed_fl = -vx * 0.866025 + vy * 0.5 + vz
    move.speed_b = -vy + vz


def compose_wheel(feedforward, correction, allow_reverse):
    if feedforward > 24.0:
        feedforward = 24.0
    elif feedforward < -24.0:
        feedforward = -24.0
    target = feedforward + correction
    if (not allow_reverse) and feedforward * target < 0.0:
        return 0.0
    limit = abs(feedforward)
    if limit < 20.0:
        limit = 20.0
    if target > limit:
        return limit
    if target < -limit:
        return -limit
    return target


def calc_follow_wheel_spd(
    move, corr_vx, corr_vy, corr_vz, ff_vx, ff_vy, ff_scale, xy_scale
):
    calc_wheel_spd(move, ff_vx * ff_scale * xy_scale, ff_vy * ff_scale * xy_scale, 0.0)
    ff_fl = move.speed_fl
    ff_fr = move.speed_fr
    ff_b = move.speed_b
    calc_wheel_spd(move, corr_vx, corr_vy, corr_vz)
    allow_reverse = ff_scale < 0.999
    move.speed_fl = compose_wheel(ff_fl, move.speed_fl, allow_reverse)
    move.speed_fr = compose_wheel(ff_fr, move.speed_fr, allow_reverse)
    move.speed_b = compose_wheel(ff_b, move.speed_b, allow_reverse)
    return (move.speed_fl + move.speed_fr + move.speed_b) / 3.0
