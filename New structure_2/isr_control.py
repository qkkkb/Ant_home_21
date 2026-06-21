class ISRControl:
    """
    ISR entry adapter.
    Keep interrupt callbacks light: only deliver data/flags.
    Put complex control logic in VehicleController.step().
    """

    def __init__(self, controller):
        self.ctrl = controller

    def pit_ch1_irq(self):
        # 5 ms periodic interrupt entry.
        self.ctrl.pit_5ms_step()

    def lpuart1_irq(self, byte_val):
        # Detection module UART byte entry.
        self.ctrl.ctx.art_detect.push(byte_val)

    def lpuart8_irq(self, byte_val):
        # Classification module UART byte entry.
        self.ctrl.ctx.art_model.push(byte_val)

    def gpio_b15_rising_irq(self, pin_level_high):
        # Page-switch state logic migrated from the GPIO interrupt.
        if not pin_level_high:
            return

        ctx = self.ctrl.ctx
        if ctx.start_flag == 1:
            if ctx.pic_num > 0:
                ctx.start_flag = 2
                ctx.show_res_flag = 1
                ctx.page_num = (ctx.pic_num - 1) // 10 + 1
        elif ctx.start_flag == 2:
            if ctx.page_num > 1:
                ctx.start_flag = 3
            ctx.show_res_flag = 1
        elif ctx.start_flag == 3:
            if ctx.page_num > 2:
                ctx.start_flag = 4
            else:
                ctx.start_flag = 2
            ctx.show_res_flag = 1
        elif ctx.start_flag == 4:
            if ctx.page_num > 3:
                ctx.start_flag = 5
            else:
                ctx.start_flag = 2
            ctx.show_res_flag = 1
        elif ctx.start_flag == 5:
            ctx.start_flag = 2
            ctx.show_res_flag = 1
