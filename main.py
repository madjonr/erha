import utime
from machine import Pin, I2C, UART
from balanceRegulator import BalanceRegulator
from imu import MPU6050
from motor import Motor
from angleFilter import Filter
from balanceCarPID import PID

regulator = BalanceRegulator()




if __name__ == '__main__':
    blue = UART(0, tx=Pin(16), rx=Pin(17), baudrate=9600)
    led = Pin(25, Pin.OUT)

    i2c = I2C(1, sda=Pin(26), scl=Pin(27), freq=400000)
    imu = MPU6050(i2c)

    motor = Motor()


    while True:
        regulator.regulateLoop()


        if blue.any() > 0:
            msg = blue.read()
            print(msg)
            if b'\xff\x01\x01\x01\x02\x00\x01\x00' == msg:
                print('forward')
                led.value(1)
            elif b'\xff\x01\x01\x01\x02\x00\x02\x00' == msg:
                print('back')
                led.value(1)
            elif b'\xff\x01\x01\x01\x02\x00\x04\x00' == msg:
                keyPressed = 'left'
                print('left')
                led.value(1)
            elif b'\xff\x01\x01\x01\x02\x00\x08\x00' == msg:
                keyPressed = 'right'
                print('right')
                led.value(1)
            elif b'\xff\x01\x01\x01\x02\x00\x00\x00' == msg:
                keyPressed = ''
                print('key up')
                led.value(0)
            else:
                print('other control')
            blue.write("recived {} \r\n".format(msg))
