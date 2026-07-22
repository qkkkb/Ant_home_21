PWM_MAX = 60000.0


def gyro_ctrl(pid, err_gyro):
    pid.err = err_gyro

    kp = pid.gyro_kp
    ki = pid.gyro_ki
    output_limit = pid.gyro_output_limit

    pid.output += kp * (pid.err - pid.err_last) + ki * pid.err
    pid.err_last = pid.err

    if pid.output > output_limit:
        pid.output = float(output_limit)
    elif pid.output < -output_limit:
        pid.output = -float(output_limit)
    return pid.output


def encoder_window(pid, sample):
    index = pid.enc_index
    total = pid.enc_sum + sample - pid.enc_samples[index]
    pid.enc_samples[index] = sample
    pid.enc_sum = total
    pid.enc_index = (index + 1) & 3
    return total * 0.25


def speed_ctrl(pid, actual_speed, tar_spd):
    pid.err = tar_spd - actual_speed
    pid.delta_tar = tar_spd - pid.tar_spd_last

    pid.delta_ud = pid.c1 * pid.delta_ud + pid.c2 * pid.delta_tar + pid.c3 * pid.delta_tar_last
    pid.output += pid.kp * (pid.err - pid.err_last) + pid.ki * pid.err + pid.delta_ud

    pid.tar_spd_last = tar_spd
    pid.delta_tar_last = pid.delta_tar
    pid.err_last = pid.err

    pwm_max = PWM_MAX
    if pid.output > pwm_max:
        pid.output = pwm_max
    elif pid.output < -pwm_max:
        pid.output = -pwm_max
    return pid.output


def speed_follow_guard(pid, output, actual_speed, target_speed, stop_eps, unload_wrong):
    if -stop_eps <= target_speed <= stop_eps:
        if actual_speed == 0:
            return 0.0
        return -pid.kp * actual_speed
    if (
        unload_wrong
        and output * target_speed < 0.0
        and actual_speed * target_speed <= target_speed * target_speed
    ):
        return 0.0
    return output


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
