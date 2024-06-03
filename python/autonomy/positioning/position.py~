from gpiozero import OutputDevice
from gpiozero import Button
import time

#dir1.on() = left motor in relative to robot camera POV
#dir2.on() = right motor in relative to robot camera POV

s1 = OutputDevice(pin=17)
d1 = OutputDevice(pin=18)
s2 = OutputDevice(pin=4)
d2 = OutputDevice(pin=27)

def calibrateZero(step1, dir1, step2, dir2):

    # ensure no direction is outward to avoid breaking mechanism
    dir1.on()
    dir2.on()

    # define limit switches
    switchRO = Button(2)
    switchRI = Button(3)
    switchLI = Button(1)
    switchLO = Button(0)

    leftcount = 0
    rightcount = 0


    #####  LEFT MOTOR  #####

    
    #extend left
    dir1.on()

    accelerate(step1, dir1, 1, 0.00003, 0.0003)
    
    while not(switchLO.is_pressed or switchLI.is_pressed or switchRI.is_pressed or switchRO.is_pressed):
        step1.on()
        step1.off()
        leftcount +=1
        time.sleep(0.0000030)

    left_position = 0 # set position left carriage

    time.sleep(1)
        
    #fold left 
    dir1.off()
    
    if(switchLI.is_pressed):

        while (left_position < leftcount) and (not(switchLO.is_pressed  or switchRI.is_pressed or switchRO.is_pressed)):
            step1.on()
            step1.off()
            left_position +=1
            time.sleep(0.0000030)

        time.sleep(1)

    #####  RIGHT MOTOR  #####

        
    #extend right   
    dir2.on()   
    
    while not(switchLO.is_pressed or switchLI.is_pressed or switchRI.is_pressed or switchRO.is_pressed):
        step2.on()
        step2.off()
        rightcount +=1

        time.sleep(0.0000030)

    right_position = 0 # set position right carriage

    time.sleep(1)

    #fold right
    dir2.off()

    if (switchRI.is_pressed):
    
        while (right_position < rightcount) and (not(switchLO.is_pressed or switchLI.is_pressed  or switchRO.is_pressed)):
            step2.on()
            step2.off()
            right_position +=1
            time.sleep(0.0000030)

        time.sleep(1)

    return left_position, right_position



    

def center(step1, dir1, step2, dir2, left_position, right_position):

    switchRO = Button(2)
    switchRI = Button(3)
    switchLI = Button(1)
    switchLO = Button(0)

    center_steps = abs(left_position - right_position)/2
    print("CENTER P")
    print(center_steps)
    
    if left_position > right_position:

        #move right
        dir1.on()
        dir2.off()
        stepcount = 0

        print("LEFT")

        while (stepcount < center_steps) and (not(switchLO.is_pressed or switchLI.is_pressed or switchRI.is_pressed or switchRO.is_pressed)):
            step1.on()
            step2.on()
            step1.off()
            step2.off()

            left_position -=1
            right_position +=1
            stepcount +=1
            
            time.sleep(0.0000030)
    

    elif right_position > left_position:

        #move left
        dir1.off()
        dir2.on()
        stepcount = 0

        print("RIGHT")

        while (stepcount < center_steps) and (not(switchLO.is_pressed or switchLI.is_pressed or switchRI.is_pressed or switchRO.is_pressed)):
            step1.on()
            step2.on()
            step1.off()
            step2.off()

            left_position -=1
            right_position +=1
            stepcount +=1
        
            time.sleep(0.0000030)

    else:
        time.sleep(1)  

    return left_position, right_position




def accelerate(step1, dir1, revs, timeDelayMin, timeDelayMax):

    switchLO = Button(2)
    switchLI = Button(3)
    switchRI = Button(1)
    switchRO = Button(0)

    stepPerRev = 3200
    accSteps = stepPerRev * revs

    count = 0
    timeDelay = timeDelayMax

    stepup = (timeDelayMax-timeDelayMin)/accSteps

    while (count < accSteps) and not(switchLO.is_pressed or switchLI.is_pressed or switchRI.is_pressed or switchRO.is_pressed):

        step1.on()
        step1.off()
        timeDelay -= stepup
        count +=1

        time.sleep(timeDelay)


def decelerate(step1, dir1, revs, timeDelayMin, timeDelayMax):

    switchLO = Button(2)
    switchLI = Button(3)
    switchRI = Button(1)
    switchRO = Button(0)

    stepPerRev = 3200
    accSteps = stepPerRev * revs

    count = 0
    timeDelay = timeDelayMin

    stepup = (timeDelayMax-timeDelayMin)/accSteps

    while (count < accSteps) and not(switchLO.is_pressed or switchLI.is_pressed or switchRI.is_pressed or switchRO.is_pressed):

        step1.on()
        step1.off()
        timeDelay += stepup
        count +=1

        time.sleep(timeDelay)


### RUN ###
#positions = calibrateZero(s1, d1, s2, d2)

#print(positions)

#time.sleep(2)

#center(s1, d1, s2, d2, positions[0], positions[1])

d1.off()
accelerate(s1, d1, 1, 0.00003, 0.0003)
decelerate(s1, d1, 1, 0.00003, 0.0003)

