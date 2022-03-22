from machine import Pin


Direction = {'FORWARD':True, 'BACKWARD':False}
MICROSTEPS: int = 16                                  # 步进电机的细分

class Motor:
    """
    马达类
    """
    def __init__(self,  dir_pin:Pin, step_pin:Pin, en_pin:Pin, side=False, pulse_period:int=20):
        self.step_pin: Pin = step_pin
        self.dir_pin: Pin = dir_pin
        self.en_pin: Pin = en_pin
        self.side = side

        self.pulse_counter: int = 0                      # 脉冲信号计数器，中断计数器？
        self.current_step_pulse: int = 0                 # 当前的脉冲位置记录点
        self.previous_step_pulse: int = 0                # 上一次的脉冲位置记录点，当前减去上一次就是期望的脉冲周期
        self.pulse_period_us: int = pulse_period         # 脉冲周期
        self.desired_step_interval = 0                   # 希望的脉冲间隔
        self.current_direction: bool = False

        self.enabled: bool = False
        

    """
    我们可以以固定的间隔（即30us、40us、50us）来执行旋转的步骤。
    所以对于我们想要达到的特定范围的合理RPS值，电机实际上会以恒定的速度旋转,
    最终导致整个数值来回跳动，并在更高的速度下产生不良振荡。
    解决办法是改变步进间隔，使其平均为合理值（并接近所需值）。
    结果是没有振荡发生，我们甚至可以增加PULSE_PERIOD。
    run()和setRPS()方法实现了这个解决方案。
    举例来说。
    - 如果期望的StepInterval等于8.2，那么第5步将在第9个（而不是第8个）脉冲后执行。
    - 或者如果期望的StepInterval等于11.9，那么10步中的9步将在第12个（而不是第11个）脉冲后执行。
    """
    def run(self):
        self.pulse_counter += 1                      
        if self.pulse_counter > self.current_step_pulse - self.previous_step_pulse:  # 这里应该计算的是马达执行一个步进后下一次执行的脉冲起始点
            self.dir_pin.value(self.current_direction ^ self.side)
            self.pulse_counter = 0
            
            self.previous_step_pulse = self.current_step_pulse
            self.current_step_pulse += self.desired_step_interval
        elif self.pulse_counter == 1:                # 创建一个步进信号
            self.step_pin.value(1)
        elif self.pulse_counter == 2:                # 创建一个步进信号
            self.step_pin.value(0)
            
    
    def setRPS(self, speed_rps):
        """
        设置马达每秒的圈数
        :param speed_rps: 期望的马达转速
        """
        speed_rps_abs = abs(speed_rps)
        if speed_rps_abs < 0.001:                        # 转速很小的情况,电机停止？
            self.desired_step_interval = 0.0
            self.current_step_pulse = 0.0
            self.previous_step_pulse = 0.0
        else:
            self.current_direction = Direction['FORWARD'] if speed_rps > 0 else Direction['BACKWARD']  # 确定马达的转向
            self.desired_step_interval = 6250/self.pulse_period_us/MICROSTEPS/speed_rps_abs            # 6250是频率，要达到期望的转速，需要的脉冲间隔
            self.current_step_pulse = self.desired_step_interval
            self.previous_step_pulse = 0
        
        
    
    def disable(self):
        self.en_pin.value(1)
        self.enabled = False
    
    def enable(self):
        self.en_pin.value(0)       
        self.enabled = True

    def forward(self):
        self.dir_pin.value(0)
        
    def back(self):
        self.dir_pin.value(1)

    

        




    
    
    
    


