from gpiozero import OutputDevice
from gpiozero.pins.pigpio import PiGPIOFactory

import time

factory = PiGPIOFactory(host='128.46.190.198')

step1 = OutputDevice(pin=23, pin_factory=factory)
direction1 = OutputDevice(pin=27, pin_factory=factory)

step2 = OutputDevice(pin=4, pin_factory=factory)
direction2 = OutputDevice(pin=21, pin_factory=factory)

while(True):
	step1.on()
	step1.off()
	time.sleep(0.000003)