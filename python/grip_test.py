from dynamixel_sdk import *
import time
sys.path.append("autonomy/gripping")

from open_grip import open_grip
from close_grip import close_grip

import time 
from gpiozero import OutputDevice
import sys, tty, termios

valve = OutputDevice(pin = 16)
cut = False

open_grip()
time.sleep(0.5)
close_grip()
time.sleep(0.5)


while not cut:
    valve.on()
    cut = input("is cut made? (y/n)")
    if cut == "y":
        cut = True
    valve.off()
    time.sleep(0.5)


