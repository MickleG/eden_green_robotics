from gpiozero import OutputDevice
import time

stepPin = 17
dirPin = 18
stepPin2 = 4
dirPin2 = 27
steppin2 = OutputDevice(pin = 4)
dirpin2 = OutputDevice(pin = 27)
steppin = OutputDevice(pin = 17)
dirpin = OutputDevice(pin = 18)



def example(speed, direction):
	if(direction == "out"):
		dirpin.on()
		dirpin2.on()
	else:
		dirpin.off()
		dirpin2.off()


	#for loop



def moveInZ(speed, direction, distance):
	if(direction == "out"):
		dirpin.on()
		dirpin2.on()
	else:
		dirpin.off()
		dirpin2.off()

	for i in range(1, distance):
		steppin.on()
		steppin2.on()
		steppin.off()
		steppin2.off()

		time.sleep(speed)

		

def moveInX(speed, direction, distance):
	if(direction == "right"):
		dirpin.on()
		dirpin2.off()
	else:
		dirpin.off()
		dirpin2. on()
	for i in range(1, distance):
		steppin.on()
		steppin2.off()
		steppin.off()
		steppin2.on()

		time.sleep(speed)

zDirection = "out"
xDirection = "left"
zDirection2 = "in"
xDirection2 = "right"

#out = 1
#in = 0
#right = 1
#left = 0


# moveInZ(.0012, zDirection)
while(True):
	moveInX(.0006, xDirection, 5001)
	moveInZ(.0006, zDirection, 2001)
	moveInX(.0006, xDirection2, 5001)
	moveInZ(.0006, zDirection2, 2001)



#movement when output(pin = 17)