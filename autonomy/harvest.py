#def lettuceKiller3000():

    # from dynamixel_sdk import *
import time
from gpiozero import OutputDevice
#import sys, tty, termios
#sys.path.append("/gripping")
#from close_grip import close_grip

valve = OutputDevice(pin = 16)
cut = False

#close_grip()
time.sleep(0.5)

while not cut:
    valve.on()
    cut = input("is cut made? (y/n)")
    if cut == "y":
        cut = True
    else:
        cut = False
    valve.off()
    time.sleep(1)
        
    






