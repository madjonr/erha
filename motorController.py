
from machine import Pin
from motor import Motor
import utime


class MotorController():
    """
    马达管理类
    """

    def __init__(self):
        self.motor_l_direction = Pin(0, Pin.OUT)
        self.motor_l_step = Pin(1, Pin.OUT)
        self.motor_l_en = Pin(5, Pin.OUT)

        self.motor_r_direction = Pin(10, Pin.OUT)
        self.motor_r_step = Pin(11, Pin.OUT)
        self.motor_r_en = Pin(15, Pin.OUT)

        self.motor_left = Motor(self.motor_l_direction, self.motor_l_step, self.motor_l_en, False)
        self.motor_right = Motor(self.motor_r_direction, self.motor_r_step, self.motor_r_en, False)

        self.enabled: bool = True

    def readyRoutine(self):
        self.setSpeed(0.2,0.2)
        utime.sleep_ms(250)
        self.setSpeed(0.0,0.0)
        utime.sleep_ms(250)
        self.setSpeed(-2.0,-0.2)
        utime.sleep_ms(250)
        self.setSpeed(0.0,0.0)
        utime.sleep_ms(250)

    def enable(self):
        self.motor_left.enable()
        self.motor_right.enable()
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


    def run(self):
        self.motor_left.run()
        self.motor_right.run()
