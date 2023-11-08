from gpiozero import LED
from gpiozero import PWMOutputDevice
from gpiozero import OutputDevice
from gpiozero import InputDevice
from gpiozero.pins.pigpio import PiGPIOFactory

import time

factory = PiGPIOFactory(host='128.46.190.249')

# led = LED(19, pin_factory=factory)
step = OutputDevice(pin=27, pin_factory=factory)
direction = OutputDevice(4, pin_factory=factory)


for i in range(10000):
	step.on()
	time.sleep(60e-6)
	step.off()
	time.sleep(60e-6)

# while True:
# 	input("Press Enter to turn LED on")
# 	led.on()
# 	input("turn it off")
# 	led.off()