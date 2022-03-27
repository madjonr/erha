
from math import radians
import utime
from angleFilter import Filter
from motorController import MotorController
from balanceCarPID import PID




ANGLE_OFFSET = 0.89                   # 角度偏移量
MAX_SPEED_CMPS = 100.0               # 最大速度：100CM/S
EXPECTED_SPEED = 0                   # 小车的期望速度，小车静止
MAX_TURN_SPEED = 0.5                 # 最大转向速度

DEAD_ANGLE = 30.0                    # 停机角度，小车超过这个角度就停机
WAKEUP_ANGLE = 1.0                   # 唤醒角度，小车在这个角度范围内就唤醒



def constrain(value, limit_min, limit_max):
    if value < limit_min:
        return limit_min
    elif value > limit_max:
        return limit_max
    else:
        return value


class BalanceRegulator():
    
    
    def __init__(self, imu):
        self.filtered_estimated_speed = 0        # 上一次预估速度过滤后的值
        self.prev_time = utime.ticks_us()        # 上一次的采样时间
        self.expected_speed = 0                  # 预期速度
        #self.angle_offset = 0                    # 偏置角度
        self.L_rps_vel = 0                       # 左边马达的转速
        self.R_rps_vel = 0                       # 右边马达的转速
        self.turn_speed = 0                      # 转向速度
        self.filter = Filter()                   # 滤波器类
        self.motors = MotorController()          # 马达控制类
        self.pid = PID(0.012, 0.0002, 0.005, 0.0002)                         # PID控制器
        self.imu = imu                           # MPU6050获取姿态数据
        self.mAverageRpsVelocity = 0             # 平均转速，中间量
        self.error_sum = 0                       # PI 计算时的累积误差
        self.prev_error_angle = 0                # 上一次的错误角度
        self.current_angle = 0                   # 当前计算的角度
        self.previous_angle = 0                  # 上一次计算的角度
    
    
    def setRelativeExpectedSpeed(self, rel_speed):
        """
        设置相对预期速度
        """
        self.expected_speed = (rel_speed - 50) / 50.0 * MAX_SPEED_CMPS
    
    
    def setTurnSpeed(self, turn_speed):
        """
        设置转向速度
        """
        self.turn_speed = turn_speed
        
    def setRelativeTurn(self, rel_turn):
        """
        设置相对转向速度
        """
        turn = (rel_turn-50)/50.0 * MAX_TURN_SPEED
        self.setTurnSpeed(turn)


    def estimateSpeed(self, dt):
        """
        估算车子的速度
        20 是轮子周长
        8.3 是轴心到MPU6050模块的距离
        """
        # 计算左右轮平均转速(圈/秒)，然后乘以车轮的周长计算小车车轮的速度
        av_cmps_vel = ((self.L_rps_vel + self.R_rps_vel) / 2.0)*20
        # 这里是通过当前的角度和前一次的角度之差除以两次之间的时间求出角速度，再乘以轮子的轴心到MPU6050模块的距离得到车的速度
        sensor_cmps_vel = radians(self.current_angle - self.previous_angle) / dt * 9.1
        # 这里是为什么？
        estimated_speed = -av_cmps_vel + sensor_cmps_vel
        #print(av_cmps_vel, sensor_cmps_vel)
        # 对速度进行平滑处理
        self.filtered_estimated_speed = 0.5 * self.filtered_estimated_speed + 0.5 * av_cmps_vel

        #return self.filtered_estimated_speed
        return av_cmps_vel
    
    
    def regulateLoop(self):
        """
        修正循环
        """
        now = utime.ticks_us()
        dt = utime.ticks_diff(now, self.prev_time)/100000                       # 求出时间间隔, 纳秒
        self.prev_time = now
        #mpu_angle = self.filter.getAngel(self.imu, dt)                       # 获取MPU6050的姿态角度
        mpu_angle = self.filter.complementary(self.imu)
        self.current_angle = mpu_angle - ANGLE_OFFSET                         # 加上偏置的角度，求得当前的实际偏离的角度
        if abs(self.current_angle) < WAKEUP_ANGLE and not self.motors.isEnabled():                            # 在唤醒角度内，唤醒马达
            self.motors.enable()
        if abs(self.current_angle) > DEAD_ANGLE and self.motors.isEnabled():                              # 超过安全角度，关闭马达
            self.motors.disable()
            self.L_rps_vel = 0
            self.R_rps_vel = 0
            self.mAverageRpsVelocity = 0
        if self.motors.enable:
            estimated_speed = self.estimateSpeed(dt)                                           # 估算小车速度
            target_angle = self.pid.PI_Speed(estimated_speed, EXPECTED_SPEED, dt)              # 计算速度环
            #print('speed:{}  angle:{}'.format(estimated_speed,target_angle))
            #regulated_delta_speed = self.pid.PD_Angel(self.current_angle, target_angle, dt)    # 计算直立环
            regulated_delta_speed = self.pid.PD_Angel(self.current_angle, ANGLE_OFFSET, dt)
            #regulated_delta_speed = constrain(regulated_delta_speed, -0.5, 0.5)               # 约束小车的加速，防止小车过冲
            #regulated_delta_speed = self.filter.filter_speed(regulated_delta_speed, 0.2)
            self.mAverageRpsVelocity += regulated_delta_speed                                  # 累积小车的速度
            self.mAverageRpsVelocity = constrain(self.mAverageRpsVelocity, -3.6, 3.6)
            #print(self.mAverageRpsVelocity, target_angle)
            self.L_rps_vel = self.mAverageRpsVelocity - target_angle - self.turn_speed                   # 左轮加上转向的速度数据
            self.R_rps_vel = self.mAverageRpsVelocity - target_angle + self.turn_speed                   # 右轮加上转向的速度数据
            print(self.mAverageRpsVelocity - target_angle)
            #print('motor_L:{}  motor_R:{}'.format(self.L_rps_vel, self.R_rps_vel))
            self.motors.setSpeed(self.L_rps_vel, self.R_rps_vel)                          # 设定左右轮的速度
            
        self.previous_angle = self.current_angle
            
            

        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        