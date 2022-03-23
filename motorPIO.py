
import rp2
from machine import Pin


Direction = {'FORWARD':0, 'BACKWARD':1}
MICROSTEPS: int = 16
STEP_P_LAP:int = 200

SPEED_MIN:float = 0
SPEED_MAX:float = 3

RREQ = 200000

@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW, out_shiftdir=rp2.PIO.SHIFT_RIGHT, autopull=True)
def blink():
    label('setXfromOSR')
    out(x, 32)
    mov(y, x)
    wrap_target()
    jmp(not_osre, 'setXfromOSR')
    set(pins, 1)
    set(pins, 0)       
    label('delay')
    nop()
    jmp(x_dec, 'delay')
    mov(x, y)
    wrap()



class Motor:
    """
    
    """        
    def __init__(self, sm_id, dir_pin:Pin, step_pin:Pin, en_pin:Pin, side=False):
        self.step_pin: Pin = step_pin
        self.dir_pin: Pin = dir_pin
        self.en_pin: Pin = en_pin
        self.side = side

        self.sm = rp2.StateMachine(sm_id, blink, freq=RREQ, set_base=step_pin)
        self.sm.active(1)


    def enable(self):
        """
        启动电机
        """
        self.sm.active(1)
        self.dir_pin.value(0)

    def disable(self):
        """
        停止电机旋转
        """
        self.sm.active(0)
        self.dir_pin.value(1)

    def speed_limits(self, speed):
        if speed < SPEED_MIN:
            return SPEED_MIN
        elif speed > SPEED_MAX:
            return SPEED_MAX
        else:
            return speed

        
    def setRPS(self, speed_rps):
        """
        设置马达每秒的圈数
        :param speed_rps: 期望的马达转速, 42电机估计只能到3，再高就容易丢步
        """
        speed_rps_abs = self.speed_limits(abs(speed_rps))
        if speed_rps_abs < 0.001:                        # 转速很小的情况,电机停止？
            self.disable()
        else:
            self.enable()
            self.current_direction = Direction['FORWARD'] if speed_rps > 0 else Direction['BACKWARD']  # 确定马达的转向
            #
            self.desired_step_interval = (RREQ-2)/(STEP_P_LAP*MICROSTEPS*speed_rps_abs)-(1+2+1)
            self.sm.put(int(self.desired_step_interval))






        




    
    
    
    


