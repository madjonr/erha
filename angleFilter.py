from math import atan, sqrt, degrees, isnan
from machine import I2C, Pin
from imu import MPU6050
import utime


RadiusToAngle = 57.2958

class Filter(object):
    """
    滤波
    """
    def __init__(self):
        self.__angle = 0.0

        self.__alpha = 0.05
        self.__x_gyro_offset = 0.0
        self.__time = utime.ticks_us()

        _i2c = I2C(1, sda=Pin(26), scl=Pin(27), freq=400000)
        self.imu = MPU6050(_i2c)


    def calculate_angle_acceleration(self, ax, ay, az):
        """
        通过加速度计算翻滚角 roll
        加速度转换成偏角乘*180/pi就是角度，不乘就是欧拉角
        """
        roll = int(degrees(atan(ay / sqrt(ax ** 2 + az ** 2 + 1e-16))))
        pitch = int(degrees(atan(-ax / sqrt(ay ** 2 + az ** 2 + 1e-16))))
        return (roll, pitch)

    @micropython.native
    def complementary(self):
        """
        互补滤波, 自身控制采样时间
        """
        accel_x, accel_y, accel_z = self.imu.accel.xyz
        gyro_x, _, _ = self.imu.gyro.xyz
        angle = degrees(atan(accel_y/sqrt(accel_x**2+accel_z**2)))
        # angle = atan(self.imu.accel.y / self.imu.accel.z)*RadiusToAngle
        __delta = utime.ticks_diff(utime.ticks_us(), self.__time) / 1000000
        self.__time = utime.ticks_us()
        self.__angle = (1 - self.__alpha) * (self.__angle + gyro_x * __delta) + self.__alpha * angle

        # output_angle = (1-self.__alpha) * (self.__angle + imu.gyro.x * self.__delta) + self.__alpha * angle
        # correction = self.constrain(imu.gyro.x, self.__x_gyro_offset-10, self.__x_gyro_offset-10)
        # self.__x_gyro_offset = self.__x_gyro_offset * 0.9995 + correction * 0.0005

        # output_angle = self.__angle * 0.5 + output_angle * 0.5
        # self.__angle = output_angle
        return self.__angle

    @micropython.native
    def getAngel(self, dt):
        """
        互补滤波，外部控制采样时间
        """
        _, accel_y, accel_z = self.imu.accel.xyz
        gyro_x, _, _ = self.imu.gyro.xyz

        # angle = atan(self.imu.accel.y / sqrt(self.imu.accel.x ** 2 + self.imu.accel.z ** 2))*RadiusToAngle
        angle = degrees(atan(accel_y  / accel_z))
        self.__angle = (1 - self.__alpha) * (self.__angle + gyro_x * dt) + self.__alpha * angle

        # correction = self.constrain(imu.gyro.x, self.__x_gyro_offset-10, self.__x_gyro_offset-10)
        # self.__x_gyro_offset = self.__x_gyro_offset * 0.9995 + correction * 0.0005

        return self.__angle


    def constrain(self, value, limit_min, limit_max):
        if value < limit_min:
            return limit_min
        elif value > limit_max:
            return limit_max
        else:
            return value

    def MPU6050_calibrate(self, imu):
        """
        校准陀螺仪
        """
        gyro_cal_ok = True
        values = []
        value = 0
        dev = 0

        utime.sleep_ms(500)
        while gyro_cal_ok:
            print("Gyro calibration... DONT MOVE!")
            # 我们在4秒内进行100次测量
            for _ in range(100):
                values.append(imu.gyro.x)
                value += imu.gyro.x
                utime.sleep_ms(25)

            value = value / 100  # 求均值
            # 计算方差
            for i in range(100):
                dev += (values[i] - value) ** 2
            dev = sqrt(dev / 100)  # 计算标准差
            print("offset:{}  stddev:{}".format(value, dev))
            if dev < 50.0:
                gyro_cal_ok = False
            else:
                print("Repeat, DONT MOVE!")

            self.__x_gyro_offset = value


# __> Kalman Filter
class Filters(object):
    def __init__(self, R: float, Q: float, alpha: float) -> None:
        self.__cov = float('nan')
        self.__x = float('nan')
        self.__c = float('nan')
        self.__A, self.__B, self.__C = 1, 0, 1
        self.__R, self.__Q = R, Q

        self.__alpha = alpha
        self.__time = utime.ticks_us()
        self.__delta = utime.ticks_diff(utime.ticks_us(), self.__time) / 1000000

    def kalman(self, angle: float) -> float:
        u = 0
        if isnan(self.__x):
            self.__x = (1 / self.__C) * angle
            self.__cov = (1 / self.__C) * self.__Q * (1 / self.__C)
        else:
            px = (self.__A * self.__x) + (self.__B * u)
            pc = ((self.__A * self.__cov) * self.__A) + self.__R
            K = pc * self.__C * (1 / ((self.__C * pc * self.__C) + self.__Q))
            self.__x = px + K * (angle - (self.__C * px))
            self.__cov = pc - (K * self.__C * pc)

        return self.__x

    def complementary(self, rate: float, angle: float) -> float:
        if isnan(self.__c):
            self.__c = angle

        self.__delta = utime.ticks_diff(utime.ticks_us(), self.__time) / 1000000
        self.__time = utime.ticks_us()
        self.__c = (1 - self.__alpha) * (self.__c + rate * self.__delta) + self.__alpha * angle
        return self.__c


