class ISRControl:
    """
    ISR 入口适配层。\n    中断回调尽量轻量：只做数据/标志投递。\n    复杂控制逻辑放在 VehicleController.step()。
    """

    def __init__(self, controller):
        self.ctrl = controller

    def pit_ch1_irq(self):
        # 5ms 周期中断入口（对应 C 工程 PIT_CH1 角色）。
        self.ctrl.pit_5ms_step()

    def lpuart1_irq(self, byte_val):
        # 检测模块串口字节入口。
        self.ctrl.ctx.art_detect.push(byte_val)

    def lpuart8_irq(self, byte_val):
        # 分类模块串口字节入口。
        self.ctrl.ctx.art_model.push(byte_val)

    def gpio_b15_rising_irq(self, pin_level_high):
        # 从 GPIO 中断迁移的页面切换状态逻辑。
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

