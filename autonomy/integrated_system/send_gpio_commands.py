from gpiozero import LED
from gpiozero.pins.pigpio import PiGPIOFactory

factory = PiGPIOFactory(host='128.46.190.249')

led = LED(26, pin_factory=factory)

while True:
	input("Press Enter to turn LED on")
	led.on()
	input("turn it off")
	led.off()

#elena was here