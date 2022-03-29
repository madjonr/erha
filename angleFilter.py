from math import atan, sqrt, degrees, isnan
import utime


class Filter(object):
    """
    滤波
    """

    def __init__(self):
        self.last_angle = 0
        self.last_speed = 0
        self.last_gyro_angle = None

        self.__angle = float('nan')

        self.__alpha = 0.05
        self.__x_gyro_offset = 0.0
        self.__time = utime.ticks_us()


    def calculate_angle_acceleration(self, ax, ay, az):
        """
        通过加速度计算翻滚角 roll
        加速度转换成偏角乘*180/pi就是角度，不乘就是欧拉角
        """
        roll = int(degrees(atan(ay / sqrt(ax ** 2 + az ** 2 + 1e-16))))
        pitch = int(degrees(atan(-ax / sqrt(ay ** 2 + az ** 2 + 1e-16))))
        return (roll, pitch)

    def complementary(self, imu):
        """
        互补滤波, 自身控制采样时间
        """
        # angle = degrees(atan(imu.accel.y/sqrt(imu.accel.x**2+imu.accel.z**2)))
        angle = degrees(atan(imu.accel.y / imu.accel.z))
        if isnan(self.__angle):
            self.__angle = angle
        __delta = utime.ticks_diff(utime.ticks_us(), self.__time) / 1000000
        self.__time = utime.ticks_us()
        self.__angle = (1 - self.__alpha) * (self.__angle + imu.gyro.x * __delta) + self.__alpha * angle

        # output_angle = (1-self.__alpha) * (self.__angle + imu.gyro.x * self.__delta) + self.__alpha * angle
        # correction = self.constrain(imu.gyro.x, self.__x_gyro_offset-10, self.__x_gyro_offset-10)
        # self.__x_gyro_offset = self.__x_gyro_offset * 0.9995 + correction * 0.0005

        # output_angle = self.__angle * 0.5 + output_angle * 0.5
        # self.__angle = output_angle
        return self.__angle

    def getAngel(self, imu, dt):
        """
        互补滤波，外部控制采样时间
        """
        # angle = degrees(atan(imu.accel.y / sqrt(imu.accel.x ** 2 + imu.accel.z ** 2)))
        angle = degrees(atan(imu.accel.y / imu.accel.z))
        if isnan(self.__angle):
            self.__angle = angle
        # self.__delta = dt
        # self.__time = utime.ticks_us()
        self.__angle = (1 - self.__alpha) * (self.__angle + imu.gyro.x * dt) + self.__alpha * angle

        # correction = self.constrain(imu.gyro.x, self.__x_gyro_offset-10, self.__x_gyro_offset-10)
        # self.__x_gyro_offset = self.__x_gyro_offset * 0.9995 + correction * 0.0005

        return self.__angle

    def kalman(self):
        return False

    def filter_angle(self, curent_angle):
        """
        一价滤波器，减少抖动
        """
        self.last_angle *= 0.8
        curent_angle = curent_angle * 0.2 + self.last_angle
        self.last_angle = curent_angle
        return curent_angle

    def filter_speed(self, curent_speed, alpha):
        """
        一价滤波器，减少抖动
        """
        curent_speed = curent_speed * alpha + self.last_speed * (1 - alpha)
        self.last_speed = curent_speed
        return curent_speed

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


