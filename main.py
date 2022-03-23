from machine import Pin, I2C, UART, Timer
from balanceRegulator import BalanceRegulator
from imu import MPU6050
from motorController import MotorController
import utime


FREQ:int = 1000
def limitFreq(freq):
    """
    Timer设置的频率不能超过0.1M，也就是100000
    """
    if freq < 1:
        return 1
    elif freq > 100000:
        return 100000
    else:
        return freq

class InvertedPendulumRobot():
    def __init__(self):
        self.blue = UART(0, baudrate=9600, tx=Pin(16), rx=Pin(17))
        self.led = Pin(25, Pin.OUT)
        _i2c = I2C(1, sda=Pin(26), scl=Pin(27), freq=400000)
        self.imu = MPU6050(_i2c)

        self.motors = MotorController()
        self.regulator = BalanceRegulator(self.imu)
        self.last_time = utime.ticks_us()

    def ISR(self, tim):
        #self.motors.run()
        now = utime.ticks_us()
        delta_time  = utime.ticks_diff(now, self.last_time)
        print(delta_time)
        self.last_time = now
        
        



if __name__ == '__main__':
    erha = InvertedPendulumRobot()

    #tim = Timer()
    #signal_freq = limitFreq(FREQ)
    # 16微秒执行一次频率就是1/0.000016=62500
    #tim.init(freq=signal_freq, mode=Timer.PERIODIC, callback=erha.ISR)

    #erha.setTimerInterrupt()
    erha.motors.readyRoutine()

    while True:
        erha.regulator.regulateLoop()

        if erha.blue.any() > 0:
            msg = erha.blue.read()
            print(msg)
            if b'\xff\x01\x01\x01\x02\x00\x01\x00' == msg:
                print('forward')
                erha.led.value(1)
            elif b'\xff\x01\x01\x01\x02\x00\x02\x00' == msg:
                print('back')
                erha.led.value(1)
            elif b'\xff\x01\x01\x01\x02\x00\x04\x00' == msg:
                keyPressed = 'left'
                print('left')
                erha.led.value(1)
            elif b'\xff\x01\x01\x01\x02\x00\x08\x00' == msg:
                keyPressed = 'right'
                print('right')
                erha.led.value(1)
            elif b'\xff\x01\x01\x01\x02\x00\x00\x00' == msg:
                keyPressed = ''
                print('key up')
                erha.led.value(0)
            else:
                print('other control')
            # erha.blue.write("recived: {} \r\n".format(msg))











