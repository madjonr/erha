from machine import Pin, UART
from balanceRegulator import BalanceRegulator
from motorController import MotorController



blue = UART(0, baudrate=9600, tx=Pin(16), rx=Pin(17))
led = Pin(25, Pin.OUT)
motors = MotorController()
regulator = BalanceRegulator()

motors.readyRoutine()
keyPressed = ''

while True:
    regulator.regulateLoop()
    if blue.any() > 0:
        msg = blue.read()
        # print(msg)
        led.value(1)
        if b'\xff\x01\x01\x01\x02\x00\x01\x00' == msg:
            keyPressed = 'forward'
            regulator.setRelativeExpectedSpeed(2)
        elif b'\xff\x01\x01\x01\x02\x00\x02\x00' == msg:
            keyPressed = 'back'
            regulator.setRelativeExpectedSpeed(-2)
        elif b'\xff\x01\x01\x01\x02\x00\x04\x00' == msg:
            keyPressed = 'left'
            regulator.setTurnTarget(0.5)
        elif b'\xff\x01\x01\x01\x02\x00\x08\x00' == msg:
            keyPressed = 'right'
            regulator.setTurnTarget(-0.5)
        elif b'\xff\x01\x01\x01\x02\x00\x00\x00' == msg:
            if keyPressed == 'left' or keyPressed == 'right':
                regulator.setTurnTarget(0)
            if keyPressed == 'forward' or keyPressed == 'back':
                regulator.setRelativeExpectedSpeed(0)
            keyPressed = 'key release'
            led.value(0)
        else:
            print('other control')
        # erha.blue.write("recived: {} \r\n".format(msg))
    # utime.sleep_ms(10)












