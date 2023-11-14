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
time_delay = 0.0000003
kp = 0.0000003
e = 0
centered = False

def runMotors():
    while(True):
        if(e > 0):
            dir1.off()
            dir2.on()
        else:
            dir1.on()
            dir2.off()

        if(not centered):
            step1.on()
            step2.on()
            step1.off()
            step2.off()
        else:
            return(1)

        time.sleep(time_delay)

def collectImage():
    global e, time_delay, centered

    while(True):
        received_msg, address = s.recvfrom(1024)
        received_msg = received_msg.decode("utf-8")
        if(";" in received_msg):
            pass
        else:
            e = int(received_msg)
            if(abs(e) <= 20):
                time_delay = 0.000003
                centered = True
            else:
                time_delay = abs(kp / e)
                centered = False

t1 = threading.Thread(target=runMotors)
t2 = threading.Thread(target=collectImage)

t1.start()
t2.start()

while(True):
    print(e)
    #message = "image_requested"
    #clientsocket.send(bytes(message, "utf-8"))
    #received_msg = clientsocket.recv(1024).decode("utf-8")
    #received_msg, address = s.recvfrom(1024)
    #received_msg = received_msg.decode("utf-8")
    #step1.on()
    #step2.on()
    #step1.off()
    #step2.off()
   # if(e == 0):
  #      time.sleep(kp)
 #   else:
#        time.sleep(abs(kp / e))

    #if(received_msg):
     #   if(";" in received_msg):
      #      pass
      #  else:
       #     e = int(received_msg)
            #print("e is {}".format(e))

        #    if(abs(e) > 20):
         #       if(e > 0):
          #          #print("error positive")
           #         dir1.off()
            #        dir2.on()
             #   elif(e < 0):
              #      #print("error negative")
               #     dir1.on()
                #    dir2.off()
