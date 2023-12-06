# towards camera's right is dir1.on() and dir2.off()
# motor1 is on the side with the 5V buck converter, motor2 is the one with the 12V buck converter

import socket
import time
from gpiozero import OutputDevice, Button
import threading
import sys

sys.path.append("../integrated_system")

from limit_switch import check_bounds

addr = "128.46.190.170"
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

time_delay = 0.0003
e_x = 0
e_z = 0
x_tol = 5
z_tol = 80
centered = False
align_x = False
align_z = False
align_y = False

received_msg = None
pull_in = False

button1 = Button(2, pull_up=True)
button2 = Button(5, pull_up=True)
#button3 = Button(3, pull_up=True)
#button4 = Button(6, pull_up=True)


### scissorsOpen = False

def runElectronics():
    while(True):
        if(not align_x and not align_z and not pull_in):
            dir1.on()
            dir2.on()
        elif(align_x and not align_z and not pull_in):
            if(e_x > 0):
                dir1.off()
                dir2.on()
            else:
                dir1.on()
                dir2.off()
        elif(not align_x and align_z and not pull_in):
            dir1.on()
            dir2.on()

        if(pull_in == True):
            dir1.off()
            dir2.off()

        if((not centered) and (received_msg)):
            step1.on()
            step2.on()
            step1.off()
            step2.off()
        #elif((align_y) and received_msg):
           # step3.on()
           # step3.off()
        ### if(scissorsOpen):
            ### gpio stuff here



        time.sleep(time_delay)

def collectMessage():
    global e_x, e_z, align_x, align_z, centered, received_msg, scissorsOpen, pull_in

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
            if(e_z < 150 and abs(e_x) > x_tol):
                align_x = True
                align_z = False
                centered = False
            #if(abs(e_x) <= 10 and align_x and not align_z and e_z > 75):
            elif((abs(e_x) <= x_tol and e_z > z_tol)):
                align_z = True
                align_x = False
                centered = False
            #if(e_z <= 75 and e_z > 50 and align_z):
            elif(e_z <= z_tol and abs(e_x) <= x_tol):
                centered = True
                time.sleep(10)
                centered = False
                pull_in = True
                time.sleep(5)

        ###elif("@" in received_msg):
            ### scissorsOpen = received_msg.split("@")[0]

t1 = threading.Thread(target = runElectronics)
t2 = threading.Thread(target = collectMessage)

t1.start()
t2.start()



while(True):
    print(received_msg)

