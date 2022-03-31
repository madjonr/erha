from micropython import const

from motorPIO import Motor
import utime


class MotorController(object):
    """
    马达管理类
    """
    def __init__(self):
        self.motor_l_direction = const(0)
        self.motor_l_step = const(1)
        self.motor_l_en = const(5)

        self.motor_r_direction = const(10)
        self.motor_r_step = const(11)
        self.motor_r_en = const(15)

        self.motor_left = Motor(0, self.motor_l_direction, self.motor_l_step, self.motor_l_en)
        self.motor_right = Motor(1, self.motor_r_direction, self.motor_r_step, self.motor_r_en)

        self.enabled: bool = True

    def readyRoutine(self):
        self.setSpeed(0.1, 0.1)
        utime.sleep_ms(250)
        self.setSpeed(0.0, 0.0)
        utime.sleep_ms(250)
        self.setSpeed(-0.1, -0.1)
        utime.sleep_ms(250)
        self.setSpeed(0.0, 0.0)

    def enable(self):
        self.motor_left.setEnable()
        self.motor_right.setEnable()
        self.enabled = True

    def disable(self):
        self.motor_left.disable()
        self.motor_right.disable()
        self.enabled = False

    def isEnabled(self):
        return self.enabled


    def setSpeed(self, motor_l_rps, motor_r_rps):
        """
        设置马达转速
        :param motor_l_rps: 左边电机的期望转速
        :param motor_r_rps: 右边电机的期望转速
        :return:
        """
        self.enable()
        self.motor_left.setRPS(motor_l_rps)
        self.motor_right.setRPS(motor_r_rps)



