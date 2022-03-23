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

_current_time = utime.ticks_ms

class PID(object):
    """A simple PID controller."""
    
    def __init__(self, speed_Kp=1.0, speed_Ki=0.0, angel_Kp=1.0, angle_Kd=0.0, sample_time=0.01, output_limits=(None, None), auto_mode=True, proportional_on_measurement=False, error_map=False):
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
        self.angel_Kp = angel_Kp
        self.angle_Kd = angle_Kd
        self.sample_time = sample_time

        self._min_output, self._max_output = None, None
        self._auto_mode = auto_mode
        self.proportional_on_measurement = proportional_on_measurement
        self.error_map = False

        self._proportional = 0                  # 比例系数
        self._integral = 0                      # 积分
        self._derivative = 0                    # 微分

        self._last_time = None                  # 最后一次采样时间
        self._last_output = None                # 最后一次输出
        self._last_input = None                 # 最后一次输入

        self.output_limits = output_limits
        self.reset()

    @property
    def output_limits(self):
        """
        The current output limits as a 2-tuple: (lower, upper).
        See also the *output_limits* parameter in :meth:`PID.__init__`.
        """
        return self._min_output, self._max_output

    @output_limits.setter
    def output_limits(self, limits):
        """Set the output limits."""
        if limits is None:
            self._min_output, self._max_output = None, None
            return

        min_output, max_output = limits

        if (None not in limits) and (max_output < min_output):
            raise ValueError('lower limit must be less than upper limit')

        self._min_output = min_output
        self._max_output = max_output

        self._integral = _clamp(self._integral, self.output_limits)
        self._last_output = _clamp(self._last_output, self.output_limits)


    def PI_Speed(self, input_, target, dt=None):
        if not self.auto_mode:                  # 如果不是自动模式就返回上一次的输出值，防止振荡。
            return self._last_output

        now = _current_time()
        if dt is None:                           # 微分时的时间步长
            dt = now - self._last_time if (now - self._last_time) else 1e-16
        elif dt <= 0:
            raise ValueError('dt has negative value {}, must be positive'.format(dt))

        if self.sample_time is not None and dt < self.sample_time and self._last_output is not None:
            # Only update every sample_time seconds
            return self._last_output

        # Compute error terms
        # 计算误差
        error = target - input_
        d_input = input_ - (self._last_input if (self._last_input is not None) else input_)  # 输入的增量

        # Check if must map the error
        if self.error_map:
            error = self.error_map(error)

        # Compute the proportional term
        if not self.proportional_on_measurement:
            # Regular proportional-on-error, simply set the proportional term
            self._proportional = self.speed_Kp * error
        else:
            # Add the proportional error on measurement to error_sum
            self._proportional -= self.speed_Kp * d_input

        # Compute integral and derivative terms
        self._integral += self.speed_Ki * error * dt
        self._integral = _clamp(self._integral, self.output_limits)  # Avoid integral windup

        #self._derivative = -self.Kd * d_input / dt

        # Compute final output
        output = self._proportional + self._integral
        output = _clamp(output, self.output_limits)

        # Keep track of state
        self._last_output = output
        self._last_input = input_
        self._last_time = now

        return output
    
    
    def PD_Angel(self, input_, target, dt=None):
        if not self.auto_mode:                  # 如果不是自动模式就返回上一次的输出值，防止振荡。
            return self._last_output

        now = _current_time()
        if dt is None:                           # 微分时的时间步长
            dt = now - self._last_time if (now - self._last_time) else 1e-16
        elif dt <= 0:
            raise ValueError('dt has negative value {}, must be positive'.format(dt))

        if self.sample_time is not None and dt < self.sample_time and self._last_output is not None:
            # Only update every sample_time seconds
            return self._last_output

        # Compute error terms
        # 计算误差
        error = target - input_
        d_input = input_ - (self._last_input if (self._last_input is not None) else input_)  # 输入的增量

        # Check if must map the error
        if self.error_map:
            error = self.error_map(error)

        # Compute the proportional term
        if not self.proportional_on_measurement:
            # Regular proportional-on-error, simply set the proportional term
            self._proportional = self.angel_Kp * error
        else:
            # Add the proportional error on measurement to error_sum
            self._proportional -= self.angel_Kp * d_input

        self._derivative = -self.angel_Kp * d_input / dt

        # Compute final output
        output = self._proportional + self._derivative
        output = _clamp(output, self.output_limits)

        # Keep track of state
        self._last_output = output
        self._last_input = input_
        self._last_time = now

        return output
        

