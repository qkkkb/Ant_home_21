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
        pid.output = 4.0 * pid.err + 0.5 * (pid.err - pid.err_last)
    else:
        if -20 < pid.err < 10:
            pid.output = 6.0 * pid.err + 0.8 * (pid.err - pid.err_last)
        if -20 < pid.err < 20:
            pid.output = 7.5 * pid.err + 1.0 * (pid.err - pid.err_last)
        else:
            pid.output = 9.0 * pid.err + 1.5 * (pid.err - pid.err_last)

    pid.err_last = pid.err
    if pid.output > 420:
        pid.output = 420.0
    elif pid.output < -420:
        pid.output = -420.0
    return pid.output


def speed_ctrl(pid, actual_speed, tar_spd):
    """
    速度环：默认使用 pid.kp / pid.ki。
    修改 kp/kd/gama 后，建议调用 pid.init_c() 重算 c1/c2/c3。
    """

    pid.err = tar_spd - actual_speed
    pid.delta_tar = tar_spd - pid.tar_spd_last

    pid.delta_ud = pid.c1 * pid.delta_ud + pid.c2 * pid.delta_tar + pid.c3 * pid.delta_tar_last
    pid.output += pid.kp * (pid.err - pid.err_last) + pid.ki * pid.err + pid.delta_ud

    pid.tar_spd_last = tar_spd
    pid.delta_tar_last = pid.delta_tar
    pid.err_last = pid.err

    pwm_max = globals().get("PWM_MAX", 24000.0)
    if pid.output > pwm_max:
        pid.output = pwm_max
    elif pid.output < -pwm_max:
        pid.output = -pwm_max
    return pid.output


def speed_reset(pid):
    pid.output = 0.0
    pid.err_last = 0.0
    pid.err = 0.0


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
