from gpiozero import OutputDevice
from gpiozero import Button
import time

#dir1.on() = left motor in relative to robot camera POV
#dir2.on() = right motor in relative to robot camera POV

#s1 = OutputDevice(pin=17)
#d1 = OutputDevice(pin=18)
#s2 = OutputDevice(pin=4)
#d2 = OutputDevice(pin=27)

def calibrateZero(step1, dir1, step2, dir2, switchRO, switchRI, switchLI, switchLO):

    # ensure no direction is outward to avoid breaking mechanism
    dir1.on()
    dir2.on()

    # define limit switches
    #switchRO = Button(2, pull_up = True)
    #switchRI = Button(3, pull_up = True)
    #switchLI = Button(1, pull_up = True)
    #switchLO = Button(0, pull_up = True)

    microsteps = 3200
    mmPerRev = 25

    leftcount = 0
    rightcount = 0

    accRevs = 0.5
    minTimeDelay = 0.00006
    maxTimeDelay = 0.0003
    accStep = accRevs * microsteps


    #####  LEFT MOTOR  #####

    
    #extend left
    dir1.on()

    accelerate(step1, dir1, accRevs, minTimeDelay, maxTimeDelay)

    left_count = accStep
    

    while not(switchLO.is_pressed or switchLI.is_pressed or switchRI.is_pressed or switchRO.is_pressed):
        step1.on()
        step1.off()
        leftcount +=1
        time.sleep(minTimeDelay)

    left_position = 0

    time.sleep(1)
        
    #fold left 
    dir1.off()
    
    while (left_position < (leftcount - (accStep))) and (not(switchLO.is_pressed  or switchRI.is_pressed or switchRO.is_pressed)):
        step1.on()
        step1.off()
        left_position += 1
        time.sleep(minTimeDelay)

    decelerate(step1, dir1, accRevs, minTimeDelay, maxTimeDelay)

    left_position += (accStep)

    time.sleep(1)

    #####  RIGHT MOTOR  #####

        
    #extend right   
    dir2.on()

    accelerate(step2, dir2, accRevs, minTimeDelay, maxTimeDelay)

    right_count = accStep  # initialize position after
    
    while not(switchLO.is_pressed or switchLI.is_pressed or switchRI.is_pressed or switchRO.is_pressed):
        step2.on()
        step2.off()
        rightcount +=1

        time.sleep(minTimeDelay)

    right_position = 0

    time.sleep(1)

    #fold right
    dir2.off()
    
    while (right_position < (rightcount - (accStep))) and (not(switchLO.is_pressed or switchLI.is_pressed  or switchRO.is_pressed)):
        step2.on()
        step2.off()
        right_position +=1
        time.sleep(minTimeDelay)

    decelerate(step2, dir2, accRevs, minTimeDelay, maxTimeDelay)

    right_position += (accStep)

    time.sleep(1)

    return left_position, right_position

    

def center(step1, dir1, step2, dir2, left_position, right_position, switchRO, switchRI, switchLI, switchLO):

    #switchRO = Button(2)
    #switchRI = Button(3)
    #switchLI = Button(1)
    #switchLO = Button(0)

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




def accelerate(step1, dir1, revs, timeDelayMin, timeDelayMax, switchLO, switchLI, switchRI, switchRO):

    #switchLO = Button(2)
    #switchLI = Button(3)
    #switchRI = Button(1)
    #switchRO = Button(0)

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


def decelerate(step1, dir1, revs, timeDelayMin, timeDelayMax, switchLO, switchLI, switchRI, switchRO):

    #switchLO = Button(2)
    #switchLI = Button(3)
    #switchRI = Button(1)
    #switchRO = Button(0)

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

def forward(step1, dir1, step2, dir2, revs):
    

    sRev = 3200
    steps = revs*sRev

    switchRO = Button(2)
    switchRI = Button(3)
    switchLI = Button(1)
    switchLO = Button(0)

    counter = 0

    while (count < steps) and not(switchLO.is_pressed or switchLI.is_pressed or switchRI.is_pressed or switchRO.is_pressed):

        step1.on()
        step2.on()
        step1.off()
        step2.off()

        delay(0.00003)


def back(step1, dir1, step2, dir2, revs, switchRO, switchRI, switchLI, switchLO):

    dir1.off()
    dir2.off()

    sRev = 3200
    steps = revs*sRev

    #switchRO = Button(2)
    #switchRI = Button(3)
    #switchLI = Button(1)
    #switchLO = Button(0)

    counter = 0

    while (count < steps) and not(switchLO.is_pressed or switchLI.is_pressed or switchRI.is_pressed or switchRO.is_pressed):

        step1.on()
        step2.on()
        step1.off()
        step2.off()

        delay(0.00003)

        


### RUN ###

 # define limit switches

#switchRO = Button(2)
#switchRI = Button(3)
#switchLI = Button(1)
#switchLO = Button(0)

#time.sleep(1)





#positions = calibrateZero(s1, d1, s2, d2)




#center1 = center(s1, d1, s2, d2, positions[0], positions[1])

#d1.on()
#acc1 = accelerate(s1, d1, 1, 0.00003, 0.0003)

#d1.on()
#acc1 = decelerate(s1, d1, 1, 0.00003, 0.0003)

#print(positions)
#print(center1)

#time.sleep(2)

#center(s1, d1, s2, d2, positions[0], positions[1])

#calibrateZero(s1, d1, s2, d2)

