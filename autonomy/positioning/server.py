import socket
import time
from gpiozero import OutputDevice

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
kp = 0.000003
e = 0

while(True):
    #message = "image_requested"
    #clientsocket.send(bytes(message, "utf-8"))
    #received_msg = clientsocket.recv(1024).decode("utf-8")
    received_msg, address = s.recvfrom(1024)
    received_msg = received_msg.decode("utf-8")
    step1.on()
    step2.on()
    step1.off()
    step2.off()
    if(e == 0):
        time.sleep(kp)
    else:
        time.sleep(abs(kp / e))

    if(received_msg):
        if(";" in received_msg):
            pass
        else:
            e = int(received_msg)
            #print("e is {}".format(e))

            if(abs(e) > 20):
                if(e > 0):
                    #print("error positive")
                    dir1.off()
                    dir2.on()
                elif(e < 0):
                    #print("error negative")
                    dir1.on()
                    dir2.off()
