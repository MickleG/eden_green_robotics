# from dynamixel_sdk import *
# import time
# import os
# from gpiozero import OutputDevice
# import sys, tty, termios
# import sys

# sys.path.append("/autonomy/gripping")

# from open_grip import open_grip
# from close_grip import close_grip

# valve = OutputDevice(pin=16)

# cut=False
# close_grip()
# time.sleep(1)

# while not cut:
#     valve.on()
#     cut = input("is cut made?(y/n)")
#     if cut == "y":
#         cut = True
#     valve.off()
#     time.sleep(0.5)    


import cv2
import time

vid = VideoCapture(0)

while True:
    ret, frame = vid.read()

    if ret:
        cv2.imshow('video', frame)

    cv2.waitKey(1)


vid.release()
cv2.destroyAllWindows()


