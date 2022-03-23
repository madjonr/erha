'''
平衡车PID控制器
'''



def _constrain(value, limit_min, limit_max):
    if value < limit_min:
        return limit_min
    elif value > limit_max:
        return limit_max
    else:
        return value


class PID(object):
    """A simple PID controller."""
    
    def __init__(self):
        self.prev_time = 0                       # 上一次的采样时间
        self.error_sum = 0                       # PI 计算时的累积误差
        self.prev_error_angle = 0                # 上一次的错误角度
        self.speed_pid_P = 1
        self.speed_pid_I = 0.1
        self.angle_pid_P = 10
        self.angle_pid_D = 0.00
        self.target_angle = 1.39                 # 小车站立的目标便置角度
        

    def PI_Speed(self, est_speed, exp_speed, dt):
        """
        速度PI环
        exp_speed 实际的估算速度
        est_speed 目标速度
        dt 积分时间
        """
        speed_error = exp_speed - est_speed
        self.error_sum += speed_error
        self.error_sum = _constrain(self.error_sum, -100000, 100000)
        output = self.speed_pid_P * speed_error + self.speed_pid_I * self.error_sum * dt
        
        return output
    
    
    def PD_Angel(self, current_angle, target_angle, dt):
        """
        直立环PD
        """
        error_angle = target_angle - current_angle                                    # 角度误差
        _derivative = self.angle_pid_D * (error_angle - self.prev_error_angle)/dt
        _derivative = _constrain(_derivative, -0.2, 0.2)      # 微分项, 进行约束
        output = self.angle_pid_P * error_angle + _derivative
        self.prev_error_angle = error_angle
        return output
        
        

