# towards camera's right is dir1.on() and dir2.off()
# motor1 is on the side with the 5V buck converter, motor2 is the one with the 12V buck converter

import socket
import time
from gpiozero import OutputDevice, Button
import threading
import sys

sys.path.append("../integrated_system")

from limit_switch import check_bounds

addr = "128.46.190.198"
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind((addr, 9000))
#s.listen(1)
print("server is now running")

#clientsocket, address = s.accept()
#print(f"Connection from {address} has been established")
print(f"Connection from {addr} has been established")
step1 = OutputDevice(pin=17)
dir1 = OutputDevice(pin=18)
step2 = OutputDevice(pin=4)
dir2 = OutputDevice(pin=27)
time_delay = 0.000003
e_x = 0
e_z = 0
centered = False
align_x = False
align_z = False

received_msg = None

button1 = Button(2)
button2 = Button(5)
button3 = Button(3)
button4 = Button(6)

### scissorsOpen = False

def runElectronics():
    while(True):
        if(check_bounds(button1, button2, button3, button4)):
           break
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

        if((not centered) and (received_msg)):
            step1.on()
            step2.on()
            step1.off()
            step2.off()
        ### if(scissorsOpen):
            ### gpio stuff here

        time.sleep(time_delay)

def collectMessage():
    global e_x, e_z, align_x, align_z, centered, received_msg, scissorsOpen

    while(True):
        received_msg, address = s.recvfrom(1024)
        received_msg = received_msg.decode("utf-8")
        if(":" in received_msg):
            pass
        elif(";" in received_msg):
            [e_x, e_z] = received_msg.split(";") # e_x in pixels, e_z in mm
            e_x = int(e_x)
            e_z = int(e_z)

            #if(e_z < 200 and not align_x and not align_z):
            if(e_z < 200 and abs(e_x) > 10):
                print("changed to align_x")
                align_x = True
                align_z = False
                centered = False
            #if(abs(e_x) <= 10 and align_x and not align_z and e_z > 75):
            elif((abs(e_x) <= 10 and e_z > 75)):
                print("changed to align_z")
                align_z = True
                align_x = False
                centered = False
            #if(e_z <= 75 and e_z > 50 and align_z):
            elif(e_z <= 75 and abs(e_x) <= 10):
                print("centered")
                centered = True
            else:
                align_x = False
                align_z = True
                centered = False
        ###elif("@" in received_msg):
            ### scissorsOpen = received_msg.split("@")[0]



t1 = threading.Thread(target=runElectronics)
t2 = threading.Thread(target=collectMessage)

t1.start()
t2.start()

while(True):
    print(e_x, e_z)

