from machine import Pin, UART
from balanceRegulator import BalanceRegulator
from motorController import MotorController
import utime


class InvertedPendulumRobot():
    def __init__(self):
        self.blue = UART(0, baudrate=9600, tx=Pin(16), rx=Pin(17))
        self.led = Pin(25, Pin.OUT)
        self.motors = MotorController()
        self.regulator = BalanceRegulator()
        self.last_time = utime.ticks_us()


if __name__ == '__main__':
    erha = InvertedPendulumRobot()
    erha.motors.readyRoutine()
    keyPressed = ''

    while True:
        erha.regulator.regulateLoop()
        if erha.blue.any() > 0:
            msg = erha.blue.read()
            # print(msg)
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
        utime.sleep_ms(10)












