PWM_MAX = 60000.0


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


def turn_ctrl(pid, err_yaw, motor_slow_flag=0):
    pid.err = err_yaw
    if pid.err > 180:
        pid.err -= 360
    elif pid.err < -180:
        pid.err += 360

    if motor_slow_flag:
        pid.output = 2.0 * pid.err + 0.3 * (pid.err - pid.err_last)
    else:
        if -10 < pid.err < 10:
            pid.output = 1.5 * pid.err + 0.3 * (pid.err - pid.err_last)
        elif -20 < pid.err < 20:
            pid.output = 2.0 * pid.err + 0.5 * (pid.err - pid.err_last)
        else:
            pid.output = 3.0 * pid.err + 0.8 * (pid.err - pid.err_last)

    pid.err_last = pid.err
    if pid.output > 15:
        pid.output = 15.0
    elif pid.output < -15:
        pid.output = -15.0
    return pid.output


def speed_ctrl(pid, actual_speed, tar_spd):
    if tar_spd == 0:
        speed_reset(pid)
        return 0

    if tar_spd * pid.tar_spd_last < 0:
        pid.output = 0.0

    error = tar_spd - actual_speed
    integral_last = pid.output
    ki = 20.0 if -7.0 <= tar_spd <= 7.0 else pid.ki
    integral = integral_last + ki * error
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
        integral = 0.0
    elif tar_spd < 0 and command > 0:
        command = 0
        integral = 0.0

    pid.err = error
    pid.err_last = error
    pid.tar_spd_last = tar_spd
    pid.output = integral
    return command


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
    pid.err_last = 0.0
    pid.err = 0.0
    pid.tar_spd_last = 0.0
    pid.delta_tar_last = 0.0
    pid.delta_tar = 0.0
    pid.delta_ud = 0.0


def pos_ctrl(err_pos):
    if -4 < err_pos < 4:
        return 2.0 * err_pos
    if err_pos > 0:
        out = 1.5 * (err_pos - 4) + 8
    else:
        out = 1.5 * (err_pos + 4) - 8

    if out > 15:
        out = 15.0
    elif out < -15:
        out = -15.0
    return out
