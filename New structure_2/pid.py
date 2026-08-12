PWM_MAX = 60000.0
_POSE_WHEEL_LIMIT = 33.0
_NORMAL_WHEEL_LIMIT = 38.0
_ALLOCATION_RESERVE = 8.0
_CORRECTION_RESERVE = 6.0


class AnglePID:
    def __init__(self):
        self.err = 0.0
        self.output = 0.0
        self.err_last = 0.0
        self.gyro_kp = 0.4
        self.gyro_ki = 0.012
        self.gyro_output_limit = 150.0


class SpeedPID:
    def __init__(self):
        self.err = 0.0
        self.tar_spd_last = 0.0
        self.output = 0.0
        self.kp = 300.0
        self.ki = 8.0


def gyro_ctrl(pid, err_gyro):
    pid.err = err_gyro

    kp = getattr(pid, "gyro_kp", 0.4)
    ki = getattr(pid, "gyro_ki", 0.012)
    output_limit = getattr(pid, "gyro_output_limit", 150.0)

    pid.output += kp * (pid.err - pid.err_last) + ki * pid.err
    pid.err_last = pid.err

    if pid.output > output_limit:
        pid.output = float(output_limit)
    elif pid.output < -output_limit:
        pid.output = -float(output_limit)
    return pid.output


def speed_ctrl(pid, actual_speed, tar_spd, hold_integral=False):
    if tar_spd == 0:
        speed_reset(pid)
        return 0

    if tar_spd * pid.tar_spd_last < 0:
        pid.output = 0.0

    error = tar_spd - actual_speed
    integral_last = pid.output
    ki = 20.0 if -7.0 <= tar_spd <= 7.0 else pid.ki
    integral = integral_last if hold_integral else integral_last + ki * error
    if integral > 18000.0:
        integral = 18000.0
    elif integral < -18000.0:
        integral = -18000.0

    command = 1450.0 * tar_spd + pid.kp * error + integral
    if command > PWM_MAX:
        command = PWM_MAX
        if error > 0:
            integral = integral_last
    elif command < -PWM_MAX:
        command = -PWM_MAX
        if error < 0:
            integral = integral_last

    if tar_spd > 0 and command < 0:
        command = 0
        if not hold_integral:
            integral = 0.0
    elif tar_spd < 0 and command > 0:
        command = 0
        if not hold_integral:
            integral = 0.0

    pid.err = error
    pid.tar_spd_last = tar_spd
    pid.output = integral
    return command


def fit_wheel_delta_scale(wfr, wfl, wb, dfr, dfl, db, limit):
    scale = 1.0
    if dfr:
        scale = ((limit if dfr > 0.0 else -limit) - wfr) / dfr
    if dfl:
        available = ((limit if dfl > 0.0 else -limit) - wfl) / dfl
        if available < scale:
            scale = available
    if db:
        available = ((limit if db > 0.0 else -limit) - wb) / db
        if available < scale:
            scale = available
    if scale < 0.0:
        return 0.0
    return 1.0 if scale > 1.0 else scale


def _wheel_targets_into(out, vx, vy, vz):
    out[0] = -vx * 0.866025 + vy * 0.5 + vz
    out[1] = vx * 0.866025 + vy * 0.5 + vz
    out[2] = -vy + vz


def _max_wheel_abs(wfr, wfl, wb):
    value = abs(wfr)
    tmp = abs(wfl)
    if tmp > value:
        value = tmp
    tmp = abs(wb)
    return tmp if tmp > value else value


def _clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def limit_pose_twist_for_wheels(
    out,
    vx,
    vy,
    vz,
    base_vx,
    base_vy,
    feedback_vx,
    feedback_vy,
    safety_vx,
    preview_vx,
    preview_vy,
    preserve_feedforward,
    preserve_rotation,
    normal_priority,
):
    limit = _NORMAL_WHEEL_LIMIT if preserve_feedforward else _POSE_WHEEL_LIMIT
    if limit <= 0.0:
        out[0] = vx
        out[1] = vy
        out[2] = vz
        out[3] = 100
        return

    if normal_priority:
        _wheel_targets_into(out, vx, vy, vz)
        if _max_wheel_abs(out[0], out[1], out[2]) <= limit:
            out[0] = vx
            out[1] = vy
            out[2] = vz
            out[3] = 100
            return

        cvx = vx - base_vx
        cvy = vy - base_vy
        preview_vx = _clamp(
            preview_vx,
            cvx if cvx < 0.0 else 0.0,
            cvx if cvx > 0.0 else 0.0,
        )
        preview_vy = _clamp(
            preview_vy,
            cvy if cvy < 0.0 else 0.0,
            cvy if cvy > 0.0 else 0.0,
        )
        corr_vx = cvx - preview_vx
        corr_vy = cvy - preview_vy
        safety_vx = _clamp(
            safety_vx,
            0.0,
            min(
                _ALLOCATION_RESERVE + _CORRECTION_RESERVE,
                corr_vx if corr_vx > 0.0 else 0.0,
            ),
        )
        core_vy = _clamp(
            corr_vy,
            -_CORRECTION_RESERVE,
            _CORRECTION_RESERVE,
        )
        core_wz = _clamp(
            vz,
            -_ALLOCATION_RESERVE,
            _ALLOCATION_RESERVE,
        )
        core_vx = safety_vx
        ox = core_vx
        oy = core_vy
        oz = core_wz
        _wheel_targets_into(out, ox, oy, oz)
        wfr = out[0]
        wfl = out[1]
        wb = out[2]
        dfr = -base_vx * 0.866025 + base_vy * 0.5
        dfl = base_vx * 0.866025 + base_vy * 0.5
        db = -base_vy
        scale = fit_wheel_delta_scale(wfr, wfl, wb, dfr, dfl, db, limit)
        ox += base_vx * scale
        oy += base_vy * scale
        wfr += dfr * scale
        wfl += dfl * scale
        wb += db * scale
        base_scale = scale
        _wheel_targets_into(out, preview_vx, preview_vy, 0.0)
        dfr = out[0]
        dfl = out[1]
        db = out[2]
        preview_scale = fit_wheel_delta_scale(
            wfr, wfl, wb, dfr, dfl, db, limit
        )
        ox += preview_vx * preview_scale
        oy += preview_vy * preview_scale
        wfr += dfr * preview_scale
        wfl += dfl * preview_scale
        wb += db * preview_scale
        yaw = vz - oz
        corr_vx -= core_vx
        corr_vy -= core_vy
        dfr = -corr_vx * 0.866025 + corr_vy * 0.5 + yaw
        dfl = corr_vx * 0.866025 + corr_vy * 0.5 + yaw
        db = -corr_vy + yaw
        scale = fit_wheel_delta_scale(wfr, wfl, wb, dfr, dfl, db, limit)
        out[0] = ox + corr_vx * scale
        out[1] = oy + corr_vy * scale
        out[2] = oz + yaw * scale
        if base_scale < 0.999:
            out[3] = -max(1, int(base_scale * 100.0))
        else:
            extra_scale = scale if scale < preview_scale else preview_scale
            out[3] = int(extra_scale * 100.0)
        return

    if preserve_rotation:
        vz = _clamp(vz, -limit, limit)
        _wheel_targets_into(out, vx, vy, 0.0)
        scale = fit_wheel_delta_scale(
            vz, vz, vz, out[0], out[1], out[2], limit
        )
        out[0] = vx * scale
        out[1] = vy * scale
        out[2] = vz
        out[3] = int(scale * 100.0)
        return

    if not preserve_feedforward:
        _wheel_targets_into(out, vx, vy, vz)
        target_max = _max_wheel_abs(out[0], out[1], out[2])
        if target_max > limit:
            scale = limit / target_max
            vx *= scale
            vy *= scale
            vz *= scale
            alloc_scale = int(scale * 100.0)
        else:
            alloc_scale = 100
        out[0] = vx
        out[1] = vy
        out[2] = vz
        out[3] = alloc_scale
        return

    corr_vx = vx - base_vx
    corr_vy = vy - base_vy
    _wheel_targets_into(out, base_vx, base_vy, 0.0)
    wheel_fr = out[0]
    wheel_fl = out[1]
    wheel_b = out[2]
    base_max = _max_wheel_abs(wheel_fr, wheel_fl, wheel_b)
    _wheel_targets_into(out, corr_vx, corr_vy, vz)
    corr_fr = out[0]
    corr_fl = out[1]
    corr_b = out[2]
    _wheel_targets_into(out, feedback_vx, feedback_vy, vz)
    reserve = _max_wheel_abs(out[0], out[1], out[2])
    if reserve > _ALLOCATION_RESERVE:
        reserve = _ALLOCATION_RESERVE
    base_limit = limit - reserve
    base_scale = 1.0
    if base_max > base_limit:
        base_scale = base_limit / base_max
        base_vx *= base_scale
        base_vy *= base_scale
        wheel_fr *= base_scale
        wheel_fl *= base_scale
        wheel_b *= base_scale
    scale = fit_wheel_delta_scale(
        wheel_fr, wheel_fl, wheel_b, corr_fr, corr_fl, corr_b, limit
    )
    out[0] = base_vx + corr_vx * scale
    out[1] = base_vy + corr_vy * scale
    out[2] = vz * scale
    out[3] = int((scale if scale < base_scale else base_scale) * 100.0)


def follow_low_pwm(cmd, target, speed_err, last_pwm, stop_eps, min_duty, smooth):
    target_zero = -stop_eps <= target <= stop_eps
    if target_zero and target == speed_err:
        return 0
    if last_pwm * cmd < 0:
        last_pwm = smooth(cmd, last_pwm)
    cmd = smooth(cmd, last_pwm)
    if target_zero and -min_duty < cmd < min_duty:
        return 0
    return cmd


def speed_reset(pid):
    pid.output = 0.0
    pid.err = 0.0
    pid.tar_spd_last = 0.0
