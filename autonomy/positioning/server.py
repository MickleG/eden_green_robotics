import socket
import time
from threading import Timer

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind(("128.46.190.198", 9000))
s.listen(5)
print("server is now running")

def background_controller():
	message = "hello!"
	clientsocket.send(bytes(message, "utf-8"))
	Timer(0.01, background_controller).start()
while(True):
	clientsocket, address = s.accept()
	print(f"Connection from {address} has been established")
	background_controller()