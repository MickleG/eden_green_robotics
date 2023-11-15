import socket

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(("128.46.190.198", 9002))

while(True):
	received_message = s.recv(1024).decode("utf-8")
	if(received_message == "image_requested"):
		s.send(bytes("ack", "utf-8"))