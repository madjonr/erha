import utime

def _clamp(value, limits):
    lower, upper = limits
    if value is None:
        return None
    elif (upper is not None) and (value > upper):
        return upper
    elif (lower is not None) and (value < lower):
        return lower
    return value


def constrain(value, limit_min, limit_max):
    if value < limit_min:
        return limit_min
    elif value > limit_max:
        return limit_max
    else:
        return value



_current_time = utime.ticks_ms

class PID(object):
    """A simple PID controller."""
    
    def __init__(self, speed_Kp=1.0, speed_Ki=0.0, angle_Kp=1.0, angle_Kd=0.0):
        """
        Initialize a new PID controller.
        param:
            Kp: P系数，控制比例增益的值
            Ki: I系数，控制积分增益的值
            Kd: D系数，控制微分增益的值
            sample_time: 采样间隔时间
            output_limits: 输出值限制范围
            auto_mode: 自动模式和手动模式
            proportional_on_measurement: 比例根据输入值计算，而不是根据误差计算
            error_map: 将误差值转换成另一个约束值的函数
        """
        self.speed_Kp = speed_Kp
        self.speed_Ki = speed_Ki
        self.angle_Kp = angle_Kp
        self.angle_Kd = angle_Kd
        self.turn_Kp = 0.1
        self.turn_Kd = 0.0001

        self._error_sum = 0                     # 积分项的误差累积和
        self.prev_error_angle = 0               # 上一次的角度误差



    def PI_Speed(self, est_speed, exp_speed, dt=None):
        """
        速度PI控制
        """
        speed_error = exp_speed - est_speed
        self._error_sum += speed_error
        # self._error_sum = constrain(self._error_sum, -1000000, 1000000)

        # Compute final output
        output = self.speed_Kp * speed_error + self.speed_Ki * self._error_sum * dt
        #output = constrain(output, -100, 100)

        return output
    
    
    def PD_Angel(self, curr_angle, target_angle, dt=None):
        # Compute error terms
        # 计算误差
        error_angle = target_angle - curr_angle
        # d_input = constrain(self.angle_Kd * (error_angle - self.prev_error_angle) / dt, -0.25, 0.25)
        d_input = self.angle_Kd * (error_angle - self.prev_error_angle) / dt
        # Compute final output
        output = self.angle_Kp * error_angle + d_input

        return output

    def PD_Turn(self, turn_target, gyro_z):
        """
        转向PD控制
        """
        if turn_target != 0:
            output = turn_target * self.turn_Kp + gyro_z * self.turn_Kd
            output = constrain(output, -0.5, 0.5)
        else:
            output = 0
        return  output


