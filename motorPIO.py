
import rp2
from machine import Pin
from micropython import const

Direction = {'FORWARD':1, 'BACKWARD':0}
MICROSTEPS = const(16)
STEP_P_LAP = const(200)

SPEED_MIN = const(0)
SPEED_MAX = const(4)

FREQ = const(200000)

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

def constrain(value, limit_min, limit_max):
    if value < limit_min:
        return limit_min
    elif value > limit_max:
        return limit_max
    else:
        return value

class Motor(object):
    """
    
    """        
    def __init__(self, sm_id, dir_pin:int, step_pin:int, en_pin:int):
        self.step_pin: Pin = Pin(step_pin, Pin.OUT)
        self.dir_pin: Pin = Pin(dir_pin, Pin.OUT, Pin.PULL_DOWN)
        self.en_pin: Pin = Pin(en_pin, Pin.OUT, Pin.PULL_DOWN)
        self.sm = rp2.StateMachine(sm_id, blink, freq=FREQ, set_base=self.step_pin)

        self.setEnable()


    def setEnable(self):
        """
        启动电机
        """
        self.sm.active(1)
        self.en_pin.value(0)

    def disable(self):
        """
        停止电机旋转
        """
        self.sm.active(0)
        self.en_pin.value(1)



    @micropython.native
    def setRPS(self, speed_rps):
        """
        设置马达每秒的圈数
        :param speed_rps: 期望的马达转速, 42电机估计只能到3，再高就容易丢步
        """
        speed_rps_abs = constrain(abs(speed_rps), -4, 4)
        dir = 1 if speed_rps > 0 else 0                      # 确定马达的转向
        self.dir_pin.value(dir)
        #
        desired_step_interval =  FREQ/(STEP_P_LAP*MICROSTEPS*speed_rps_abs+1e-16) -(2+1+1+1+1)
        self.sm.put(int(desired_step_interval))






        




    
    
    
    


