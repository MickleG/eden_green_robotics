from gpiozero import OutputDevice
import time

step1 = OutputDevice(pin=17)
dir1 = OutputDevice(pin=18)
step2 = OutputDevice(pin=4)
dir2 = OutputDevice(pin=27)

while(True):
    step1.on()
    step2.on()
    step1.off()
    step2.off()
    time.sleep(0.000003)
