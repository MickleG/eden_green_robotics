from gpiozero import OutputDevice
from gpiozero import Button

import time

#dir1.on() = right motor in
#dir2.on() = left motor in

microsteps = 3200
mmPerRev = 25

#time.sleep(.1)

step1 = OutputDevice(pin=17)
dir1 = OutputDevice(pin=18)
step2 = OutputDevice(pin=4)
dir2 = OutputDevice(pin=27)

#switchLO = Button(2, pull_up = True)
#switchLI = Button(3, pull_up = True)
#switchRI = Button(1, pull_up = True)
#switchRO = Button(0, pull_up = True)


def calibrate():

    left_motor = 0
    right_motor = 0
    centersteps = 0

    # move left
    dir1.on()
    dir2.off()

    while not(switchLO.is_pressed or switchLI.is_pressed or switchRI.is_pressed or switchRO.is_pressed):
        step1.on()
        step2.on()
        step1.off()
        step2.off()

        time.sleep(0.0000030)

    time.sleep(1)

    # move right
    dir1.off()
    dir2.on()

    if switchLO.is_pressed:
        left_motor = -32000

        print("Left motor collision")

        while not(switchRO.is_pressed or switchRI.is_pressed or switchLI.is_pressed):
            step1.on()
            step2.on()
            step1.off()
            step2.off()

            time.sleep(0.0000030)

            left_motor += 1

        right_motor = 32000


    elif switchRI.is_pressed:
        right_motor = 0

        print("Right motor Collision")

        while not(switchLI.is_pressed or switchLO.is_pressed or switchRO.is_pressed):
            step1.on()
            step2.on()
            step1.off()
            step2.off()

            time.sleep(0.0000030)

            right_motor += 1

        left_motor = 0

    print(left_motor)
    print("\n")
    print(right_motor)
    

    #move left
    dir1.on()
    dir2.off()

    centersteps = abs((left_motor + right_motor)/2)

    print(centersteps)

    while centersteps > 0 and not(switchRI.is_pressed or switchLO.is_pressed):
            step1.on()
            step2.on()
            step1.off()
            step2.off()

            time.sleep(0.0000030)

            centersteps -= 1
            right_motor -= 1
            left_motor -= 1

    return left_motor, right_motor

time.sleep(.1)

steps = 0
dir1.on()

while steps < 3200:
    step1.on()
    step1.off()
    steps += 1

    time.sleep(0.00003)

#calibrate()






