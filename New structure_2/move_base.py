BODY_X_SIGN = -1.0


def calc_wheel_spd(move, vx, vy, vz):
    vx = BODY_X_SIGN * vx
    move.speed_fr = vx * 0.866025 + vy * 0.5 + vz
    move.speed_fl = -vx * 0.866025 + vy * 0.5 + vz
    move.speed_b = -vy + vz


def _compose_wheel(feedforward, correction, ff_limit, target_limit, allow_reverse):
    if feedforward > ff_limit:
        feedforward = ff_limit
    elif feedforward < -ff_limit:
        feedforward = -ff_limit
    target = feedforward + correction
    if (not allow_reverse) and feedforward * target < 0.0:
        return 0.0
    feedforward = abs(feedforward)
    if feedforward > target_limit:
        target_limit = feedforward
    if target > target_limit:
        return target_limit
    if target < -target_limit:
        return -target_limit
    return target


def calc_wheel_spd_2dof(
    move,
    ff_vx,
    ff_vy,
    ff_wz,
    corr_vx,
    corr_vy,
    corr_vz,
    wz_to_vx,
    wz_to_vy,
    forward_gain,
    lateral_gain,
    forward_limit,
    lateral_limit,
    xy_scale,
    ff_scale,
    ff_limit,
    target_limit,
    allow_reverse,
):
    ff_vx = (ff_vx + ff_wz * wz_to_vx) * forward_gain
    ff_vy = (ff_vy + ff_wz * wz_to_vy) * lateral_gain
    if ff_vx > forward_limit:
        ff_vx = forward_limit
    elif ff_vx < -forward_limit:
        ff_vx = -forward_limit
    if ff_vy > lateral_limit:
        ff_vy = lateral_limit
    elif ff_vy < -lateral_limit:
        ff_vy = -lateral_limit
    ff_vx *= xy_scale * ff_scale
    ff_vy *= xy_scale * ff_scale
    ff_vx = BODY_X_SIGN * ff_vx
    corr_vx = BODY_X_SIGN * corr_vx
    move.speed_fr = _compose_wheel(
        ff_vx * 0.866025 + ff_vy * 0.5,
        corr_vx * 0.866025 + corr_vy * 0.5 + corr_vz,
        ff_limit,
        target_limit,
        allow_reverse,
    )
    move.speed_fl = _compose_wheel(
        -ff_vx * 0.866025 + ff_vy * 0.5,
        -corr_vx * 0.866025 + corr_vy * 0.5 + corr_vz,
        ff_limit,
        target_limit,
        allow_reverse,
    )
    move.speed_b = _compose_wheel(
        -ff_vy,
        -corr_vy + corr_vz,
        ff_limit,
        target_limit,
        allow_reverse,
    )
    return (move.speed_fl + move.speed_fr + move.speed_b) / 3.0
