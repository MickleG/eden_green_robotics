from dynamixel_sdk import *
import time
import os
from gpiozero import OutputDevice
import sys, tty, termios
import sys

sys.path.append("/autonomy/gripping")

from open_grip import open_grip
from close_grip import close_grip

valve = OutputDevice(pin=15)

cut=False
close_grip()
time.sleep(1)

while cut = False
    valve.on()
    cut = input("is cut made?(y/n)")
    if cut == "y":
open_grip()


