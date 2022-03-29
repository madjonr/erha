from machine import Pin, I2C, UART
from balanceRegulator import BalanceRegulator
from imu import MPU6050
from motorController import MotorController
import utime


class InvertedPendulumRobot():
    def __init__(self):
        self.blue = UART(0, baudrate=9600, tx=Pin(16), rx=Pin(17))
        self.led = Pin(25, Pin.OUT)
        _i2c = I2C(1, sda=Pin(26), scl=Pin(27), freq=400000)
        self.imu = MPU6050(_i2c)
        self.motors = MotorController()
        self.regulator = BalanceRegulator(self.imu)
        self.last_time = utime.ticks_us()


if __name__ == '__main__':
    erha = InvertedPendulumRobot()
    erha.motors.readyRoutine()

    # sw = Pin(6, Pin.IN)
    # sw.irq(trigger=Pin.IRQ_FALLING, handler=test())

    keyPressed = ''
    while True:
        erha.regulator.regulateLoop()
        if erha.blue.any() > 0:
            msg = erha.blue.read()
            erha.led.value(1)
            if b'\xff\x01\x01\x01\x02\x00\x01\x00' == msg:
                keyPressed = 'forward'
                erha.regulator.setRelativeExpectedSpeed(5.0)
            elif b'\xff\x01\x01\x01\x02\x00\x02\x00' == msg:
                keyPressed = 'back'
                erha.regulator.setRelativeExpectedSpeed(-5.0)
            elif b'\xff\x01\x01\x01\x02\x00\x04\x00' == msg:
                keyPressed = 'left'
                erha.regulator.setTurnTarget(1)
            elif b'\xff\x01\x01\x01\x02\x00\x08\x00' == msg:
                keyPressed = 'right'
                erha.regulator.setTurnTarget(-1)
            elif b'\xff\x01\x01\x01\x02\x00\x00\x00' == msg:
                if keyPressed == 'left' or keyPressed == 'right':
                    erha.regulator.setTurnTarget(0)
                if keyPressed == 'forward' or keyPressed == 'back':
                    erha.regulator.setRelativeExpectedSpeed(0)
                keyPressed = 'key release'
                erha.led.value(0)
            else:
                print('other control')
            # erha.blue.write("recived: {} \r\n".format(msg))












