
import utime
from angleFilter import Filter
from motorController import MotorController
from balanceCarPID import PID




ANGLE_OFFSET = 0.67                  # 角度偏移量
MAX_SPEED_CMPS = 80.0                # 最大速度：100CM/S
EXPECTED_SPEED = 0                   # 小车的期望速度，小车静止
MAX_TURN_SPEED = 0.3                 # 最大转向速度

DEAD_ANGLE = 90.0                    # 停机角度，小车超过这个角度就停机
WAKEUP_ANGLE = 1.0                   # 唤醒角度，小车在这个角度范围内就唤醒



def constrain(value, limit_min, limit_max):
    if value < limit_min:
        return limit_min
    elif value > limit_max:
        return limit_max
    else:
        return value



class BalanceRegulator():

    def __init__(self):
        self.filtered_estimated_speed = 0        # 上一次预估速度过滤后的值
        self.prev_time = utime.ticks_us()        # 上一次的采样时间
        self.expected_speed = 0                  # 预期速度
        #self.angle_offset = 0                    # 偏置角度
        self.L_rps_vel = 0                       # 左边马达的转速
        self.R_rps_vel = 0                       # 右边马达的转速
        self.turn_speed = 0                      # 转向速度
        self.filter = Filter()                   # 滤波器类
        self.motors = MotorController()          # 马达控制类
        self.pid = PID(0.012, 0.0002, 0.0035, 0.00018)                         # PID控制器
        self.mAverageRpsVelocity = 0             # 平均转速，中间量
        self.error_sum = 0                       # PI 计算时的累积误差
        self.prev_error_angle = 0                # 上一次的错误角度
        self.current_angle = 0                   # 当前计算的角度
        self.previous_angle = 0                  # 上一次计算的角度
        self.turn_target = 0.0

        self.turn_speed_delta = 0

    def setTurnTarget(self, target):
        """
        设置转向速度
        """
        self.turn_target = target
        self.turn_speed_delta = 0

    def setExpectedSpeed(self, direction_angle):
        self.angle_offset = direction_angle + ANGLE_OFFSET


    def estimateSpeed(self):
        """
        估算车子的速度
        20 是轮子周长
        8.3 是轴心到MPU6050模块的距离
        """
        av_cmps_vel = ((self.L_rps_vel + self.R_rps_vel) / 2.0) * 20
        # self.filtered_estimated_speed = 0.5 * self.filtered_estimated_speed + 0.5 * av_cmps_vel
        # return self.filtered_estimated_speed
        return av_cmps_vel

    
    def regulateLoop(self):
        """
        修正循环
        """
        now = utime.ticks_us()
        dt = utime.ticks_diff(now, self.prev_time)/1000000                       # 求出时间间隔, 纳秒
        self.prev_time = now
        mpu_angle = self.filter.complementary()
        # print(dt*1000000)
        self.current_angle = mpu_angle - ANGLE_OFFSET                                     # 加上偏置的角度，求得当前的实际偏离的角度
        # self.current_angle = self.previous_angle*0.5 + current_angle*0.5                # 减去偏置的角度，求得当前的实际偏离的角度
        if abs(self.current_angle) < WAKEUP_ANGLE and not self.motors.enabled:            # 在唤醒角度内，唤醒马达
            self.motors.enable()
        if abs(self.current_angle) > DEAD_ANGLE and self.motors.enabled:                  # 超过安全角度，关闭马达
            self.motors.disable()
            self.L_rps_vel = 0
            self.R_rps_vel = 0
            self.mAverageRpsVelocity = 0
        if self.motors.enabled:
            estimated_speed = self.estimateSpeed()                                         # 估算小车速度
            target_speed = self.pid.PI_Speed(estimated_speed, self.expected_speed, dt)         # 计算速度环
            self.mAverageRpsVelocity += self.pid.PD_Angel(self.current_angle, ANGLE_OFFSET, dt)     # 计算直立环，累积小车的速度
            self.mAverageRpsVelocity = constrain(self.mAverageRpsVelocity, -3, 3)
            # self.turn_speed_delta = self.pid.PD_Turn(self.turn_target, self.imu.gyro.z)         # 计算转向环                                        # 累积小车的转向速度
            self.L_rps_vel = self.mAverageRpsVelocity  + self.turn_speed_delta   # 左轮加上转向的速度数据
            self.R_rps_vel = self.mAverageRpsVelocity  - self.turn_speed_delta   # 右轮加上转向的速度数据
            self.motors.setSpeed(self.L_rps_vel, self.R_rps_vel)                               # 设定左右轮的速度
        print(self.current_angle, self.L_rps_vel)
        #self.previous_angle = current_angle
            
            

        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        