

from math import radians
import utime
from angleFilter import Filter
from imu import MPU6050
from motor import Motor
from balanceCarPID import PID




ANGLE_OFFSET = 1.39                  # 角度偏移量
MAX_SPEED_CMPS = 100.0               # 最大速度：100CM/S
EXPECTED_SPEED = 0                   # 小车的期望速度，小车静止
MAX_TURN_SPEED = 1.0                 # 最大转向速度

DEAD_ANGLE = 50.0                    # 停机角度，小车超过这个角度就停机
WAKEUP_ANGLE = 1.0                   # 唤醒角度，小车在这个角度范围内就唤醒



def _constrain(self, value, limit_min, limit_max):
    if value < limit_min:
        return limit_min
    elif value > limit_max:
        return limit_max
    else:
        return value


class BalanceRegulator():
    
    
    def __init__(self, imu):
        self.filtered_estimated_speed = 0        # 上一次预估速度过滤后的值
        self.prev_time = 0                       # 上一次的采样时间
        self.expected_speed = 0                  # 预期速度
        #self.angle_offset = 0                    # 偏置角度
        self.L_rps_vel = 0
        self.R_rps_vel = 0
        self.turn_speed = 0                      # 转向速度
        self.filter = Filter()
        self.motor = Motor()
        self.pid = PID()
        self.imu = imu
        self.mAverageRpsVelocity = 0             # 平均fps速度，后面要看看*****
        self.error_sum = 0                       # PI 计算时的累积误差
        self.prev_error_angle = 0                # 上一次的错误角度
        self.current_angle = 0                   # 当前计算的角度
        self.previous_angle = 0                  # 上一次计算的角度
    
    
    def setRelativeExpectedSpeed(self, rel_speed):
        """
        设置相对预期速度, 后面可改成属性的形式
        """
        self.expected_speed = (rel_speed - 50) / 50.0 * MAX_SPEED_CMPS
    
    
    def setTurnSpeed(self, turn_speed):
        """
        设置转向速度
        """
        self.turn_speed = turn_speed
        
    def setRelativeTurn(self, rel_turn):
        """
        设置相对转向
        """
        turn = (rel_turn-50)/50.0 * MAX_TURN_SPEED
        self.setTurnSpeed(turn)
    
    
    def estimateSpeed(self, dt):
        """
        估算车子的速度
        20 是轮子周长
        8.3 是轴心到MPU6050模块的距离
        
        """
        # 这里是计算平均左右轮走了多少圈，然后计算行走的距离？
        av_cmps_vel = ((self.L_rps_vel + self.R_rps_vel) / 2.0) * 20
        # 这里是通过当前的角度和前一次的角度计算之差除以两次计算之间的时间求出角速度，再乘以轮子的轴心到MPU6050模块的距离得到车的速度        
        sensor_cmps_vel = radians(self.current_angle - self.previous_angle) / dt * 8.3
        estimated_speed = -av_cmps_vel + sensor_cmps_vel
        
        # 这里是通过当前的角度和前一次的角度计算之差除以两次计算之间的时间求出角速度，再乘以轮子的轴心到MPU6050模块的距离得到车的边缘速度。
        self.filtered_estimated_speed = 0.5 * self.filtered_estimated_speed + 0.5 * estimated_speed
        
        return self.filtered_estimated_speed
    
    
    def regulateLoop(self, imu):
        """
        修正循环
        """
        now = utime.ticks_us()
        dt = utime.ticks_diff(utime.ticks_us(), self.prev_time)/1000000
        self.prev_time = now
        mpu_angle = self.filter.getAngle(self.imu, dt)
        current_angle = mpu_angle + ANGLE_OFFSET
        
        if abs(current_angle) < WAKEUP_ANGLE:     # 在唤醒角度内，唤醒马达
            self.motor.enable()
        if abs(current_angle) > DEAD_ANGLE:       # 超过安全角度，关闭马达
            self.motor.disable()
            L_rps_vel = 0
            R_lps_vel = 0
            mAverageRpsVelocity = 0
            
        if self.motor.enable:
            estimated_speed = self.estimateSpeed(dt)   # 估算速度
            target_angle = self.pid.PI_Speed(estimated_speed, EXPECTED_SPEED, dt)
            regulated_delta_speed = self.pid.PD_Angel(current_angle, target_angle)
            
            #regulated_delta_speed = _constrain(regulated_delta_speed, -5.0, 5.0)   # 约束小车的加速
            
            self.mAverageRpsVelocity += regulated_delta_speed
            
            L_rps_vel = self.mAverageRpsVelocity - self.turn_speed
            R_rps_vel = self.mAverageRpsVelocity + self.turn_speed
            
            self.motor.set_speed(L_rps_vel, R_rps_vel)
            
        self.previous_angle = current_angle
            
            

        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        