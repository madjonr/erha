from machine import Pin, PWM
from micropython import const


MICROSTEPS = const(16)
STEP_P_LAP = const(200)
SPEED_LIMIT = (-4, 4)
FREQ_LIMIT = (15, 7500)
DUTY_CYCLE = const(33023)                 # 2**16//2 + 255


def constrain(value, limit_min, limit_max):
    if value < limit_min:
        return limit_min
    elif value > limit_max:
        return limit_max
    else:
        return value


class Motor(object):
    """

    """
    def __init__(self, dir_pin: int, step_pin: int, en_pin: int):
        self.step_pin: PWM = PWM(Pin(step_pin, Pin.OUT))
        self.dir_pin: Pin = Pin(dir_pin, Pin.OUT, Pin.PULL_DOWN)
        self.en_pin: Pin = Pin(en_pin, Pin.OUT, Pin.PULL_DOWN)

        self.step_pin.duty_u16(DUTY_CYCLE)
        self.setEnable()

    def setEnable(self):
        """
        启动电机
        """
        self.en_pin.value(0)
        self.step_pin.duty_u16(DUTY_CYCLE)

    def disable(self):
        """
        停止电机旋转
        """
        self.en_pin.value(1)
        self.step_pin.duty_u16(0)

    @micropython.native
    def setRPS(self, speed_rps):
        """
        设置马达每秒的圈数
        :param speed_rps: 期望的马达转速, 42电机估计只能到3，再高就容易丢步
        """
        speed_rps_abs = constrain(abs(speed_rps), -4, 4)
        dir = 1 if speed_rps > 0 else 0  # 确定马达的转向
        self.dir_pin.value(dir)
        #
        desired_step_interval = STEP_P_LAP * MICROSTEPS * speed_rps_abs
        desired_step_interval = constrain(desired_step_interval, FREQ_LIMIT[0], FREQ_LIMIT[1])
        self.step_pin.freq(int(desired_step_interval))

















