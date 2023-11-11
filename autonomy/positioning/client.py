import socket

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# s.connect(("128.46.190.198", 9000))
s.connect(("10.211.55.3", 9000))

while(True):
	print(s.recv(1024).decode("utf-8"))