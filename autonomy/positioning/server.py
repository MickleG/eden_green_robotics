# towards camera's right is dir1.on() and dir2.off()
# motor1 is on the side with the 5V buck converter, motor2 is the one with the 12V buck converter

import socket
import time
from gpiozero import OutputDevice
import threading

addr = "128.46.190.198"
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind((addr, 9000))
#s.listen(1)
print("server is now running")

#clientsocket, address = s.accept()
#print(f"Connection from {address} has been established")
print(f"Connection from {addr} has been established")
step1 = OutputDevice(pin=23)
dir1 = OutputDevice(pin=27)
step2 = OutputDevice(pin=4)
dir2 = OutputDevice(pin=21)
time_delay = 0.000003
e_x = 0
e_z = 0
centered = False
align_x = False
align_z = False

received_msg = None
dummy = True


def runMotors():
    while(True):
        if(not align_x and not align_z):
            dir1.on()
            dir2.on()
        elif(align_x and not align_z):
            if(e_x > 0):
                dir1.off()
                dir2.on()
            else:
                dir1.on()
                dir2.off()
        elif(not align_x and align_z):
            dir1.on()
            dir2.on()
        #if(e_x > 0 and align_x and not align_z):
         #   dir1.off()
          #  dir2.on()
        #elif(e_x <= 0 and align_x):
         #   dir1.on()
          #  dir2.off()

        if((not centered) and (received_msg)):
            step1.on()
            step2.on()
            step1.off()
            step2.off()

        time.sleep(time_delay)

def collectImage():
    global e_x, e_z, align_x, align_z, centered, received_msg

    while(True):
        received_msg, address = s.recvfrom(1024)
        received_msg = received_msg.decode("utf-8")
        if(":" in received_msg):
            pass
        elif(";" in received_msg):
            [e_x, e_z] = received_msg.split(";") # e_x in pixels, e_z in mm
            e_x = int(e_x)
            e_z = int(e_z)

            if(e_z < 200 and not align_x and not align_z):
                print("changed to align_x")
                align_x = True
                align_z = False
            if(abs(e_x) <= 20 and align_x and not align_z):
                print("changed to align_z")
                align_z = True
                align_x = False
            if(e_z <= 65 and align_z):
                print("centered")
                centered = True

t1 = threading.Thread(target=runMotors)
t2 = threading.Thread(target=collectImage)

t1.start()
t2.start()

while(True):
    print(e_z)

