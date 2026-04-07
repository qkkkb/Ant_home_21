BLOCK_LEFT = 0
BLOCK_RIGHT = 1


def clear_data_fifo(uart):
    uart.pop_all()


def deal_art_correct(ctx, uart):
    if uart.used() == 0:
        return 0
    data = uart.pop_all()
    if len(data) != 2:
        return 0

    ctx.picture_x = int(data[0]) - 120
    ctx.picture_y = int(data[1]) - 120

    if ctx.picture_y > 60 or ctx.picture_y < -60:
        ctx.move.tar_spd_x = ctx.picture_y * 0.5
        ctx.move.tar_spd_y = -ctx.picture_x * 0.45
    else:
        ctx.move.tar_spd_x = ctx.picture_y * 0.3
        ctx.move.tar_spd_y = -ctx.picture_x * 0.27

    if ctx.move.tar_spd_x > 40:
        ctx.move.tar_spd_x = 40
    elif ctx.move.tar_spd_x < -40:
        ctx.move.tar_spd_x = -40
    return 1


def deal_art_correct_y(ctx, uart):
    if uart.used() == 0:
        return 0
    data = uart.pop_all()
    if len(data) != 2:
        return 0

    ctx.picture_x = int(data[0]) - 120
    ctx.picture_y = int(data[1]) - 120
    ctx.move.tar_spd_y = -ctx.picture_x * 0.42
    return 1


def deal_art_detect(ctx, uart):
    if uart.used() == 0:
        return
    data = uart.pop_all()
    if len(data) != 1:
        return

    marker = data[0]
    if marker == 250:
        ctx.move.tar_spd_x = ctx.slow_trace_speed
        ctx.motor_slow_flag = 1
    elif marker == 251:
        ctx.curve_angle_flag = -1
    elif marker == 252:
        ctx.curve_angle_flag = 1


def deal_art_class(ctx, uart, push_from_label):
    if uart.used() == 0:
        return 0
    data = uart.pop_all()
    if len(data) != 1:
        return 0

    cls = int(data[0])
    if ctx.pic_num < len(ctx.class_type):
        ctx.class_type[ctx.pic_num] = cls

    if cls < 15:
        ctx.push_direction = push_from_label(cls)
    else:
        if cls % 2:
            ctx.push_direction = BLOCK_LEFT
        else:
            ctx.push_direction = BLOCK_RIGHT

    ctx.pic_num += 1
    return 1
