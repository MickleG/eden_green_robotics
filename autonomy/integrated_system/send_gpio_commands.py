#direction1.on() and direction2.off() goes to robot's right
from gpiozero import OutputDevice
import time

step1 = OutputDevice(pin=23)
direction1 = OutputDevice(pin=27)

step2 = OutputDevice(pin=4)
direction2 = OutputDevice(pin=21)

while(True):
    step1.on()
    step1.off()
    step2.on()
    step2.off()
    direction1.on()
    direction2.off()
    time.sleep(0.000003)
