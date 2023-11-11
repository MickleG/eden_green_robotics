import socket
import time
from threading import Timer

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind(("", 9000))
s.listen(5)
print("server is now running")

def background_controller():
	message = str(time.time_ns())
	print(message)
	clientsocket.send(bytes(message, "utf-8"))

clientsocket, address = s.accept()
print(f"Connection from {address} has been established")

while(True):
	background_controller()
	time.sleep(0.1)
