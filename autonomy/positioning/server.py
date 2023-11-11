import socket
import time
from threading import Timer

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
hostname = socket.gethostname()
s.bind((hostname, 9000))
print(hostname)
print(socket.gethostbyname(hostname))
s.listen(5)
print("server is now running")

def background_controller():
	message = "hello!"
	clientsocket.send(bytes(message, "utf-8"))
	Timer(5, background_controller).start()
while(True):
	clientsocket, address = s.accept()
	print(f"Connection from {address} has been established")
	background_controller()