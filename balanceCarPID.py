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
        self.speed_Kp = speed_Kp
        self.speed_Ki = speed_Ki
        self.angle_Kp = angle_Kp
        self.angle_Kd = angle_Kd
        self.turn_Kp = 0.1
        self.turn_Kd = 0.0001

        self._error_sum = 0                     # 积分项的误差累积和
        self.prev_error_angle = 0               # 上一次的角度误差
        self.prev_turn_speed = 0                # 上一次的转向PD输出

    @micropython.native
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

    @micropython.native
    def PD_Angel(self, curr_angle, target_angle, dt=None):
        # Compute error terms
        # 计算误差
        error_angle = target_angle - curr_angle
        # d_input = constrain(self.angle_Kd * (error_angle - self.prev_error_angle) / dt, -0.25, 0.25)
        d_input = self.angle_Kd * (error_angle - self.prev_error_angle) / dt
        # Compute final output
        output = self.angle_Kp * error_angle + d_input

        return output

    @micropython.native
    def PD_Turn(self, current_turn, turn_target, dt):
        """
        转向PD控制
        """
        if turn_target == 0:
            return 0
        error_turn = turn_target - current_turn
        d_input = self.turn_Kd * (error_turn - self.prev_turn_speed) / dt
        output = error_turn * self.turn_Kp + d_input
        return  output


