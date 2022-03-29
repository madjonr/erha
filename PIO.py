import rp2
from machine import Pin

FREQ = 200000

@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
def blink():
    wrap_target()
    set(pins, 1)   [31]
    set(pins, 0)   [31]
    wrap()


class PIO():
    def __init__(self):
        self.sm = rp2.StateMachine(3, blink, freq=FREQ, set_base=Pin(6))
        self.sm.active(1)