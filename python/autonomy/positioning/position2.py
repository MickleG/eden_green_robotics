from gpiozero import OutputDevice
from gpiozero import Button
import time
import math

#dir1.on() = left motor in relative to robot camera POV
#dir2.on() = right motor in relative to robot camera POV

#left_motor / right_motor are step counts, 0 = extended and high count = folded

def calibrateZero(step1, dir1, step2, dir2, switchLO, switchLI, switchRI, switchRO):

    microsteps = 3200
    mmPerRev = 25

    # ensure no direction is outward to avoid breaking mechanism
    dir1.on()
    dir2.on()

    leftcount = 0
    rightcount = 0

    accRevs = 0.5
    minTimeDelay = 0.000003
    maxTimeDelay = 0.0003
    accStep = accRevs * microsteps


    #####  LEFT MOTOR  #####

    
    #extend left
    dir1.on()

    leftcount = accelerateSingle(step1, dir1, switchLO, switchLI, switchRI, switchRO, accRevs, minTimeDelay, maxTimeDelay, leftcount)
    leftcount *= -1

    print(leftcount)
    
    while not(switchLO.is_pressed or switchLI.is_pressed or switchRI.is_pressed or switchRO.is_pressed):
        step1.on()
        step1.off()
        leftcount +=1
        time.sleep(minTimeDelay)

        if switchLO.is_pressed:
            print("LO")

        if switchLI.is_pressed:
            print("LI")

        if switchRO.is_pressed:
            print("RO")

        if switchRI.is_pressed:
            print("RI")

    left_position = 0

    time.sleep(0.5)
        
    #fold left 
    dir1.off()
    
    while (left_position < (leftcount - accStep)) and (not(switchLO.is_pressed or switchRI.is_pressed or switchRO.is_pressed)):
        step1.on()
        step1.off()
        left_position += 1
        time.sleep(minTimeDelay)

    left_position = decelerateSingle(step1, dir1, switchLO, switchLI, switchRI, switchRO, accRevs, minTimeDelay, maxTimeDelay, left_position)

    time.sleep(0.5)

    #####  RIGHT MOTOR  #####



        
    #extend right   
    dir2.on()

    rightcount = accelerateSingle(step2, dir2, switchLO, switchLI, switchRI, switchRO, accRevs, minTimeDelay, maxTimeDelay, rightcount)
    rightcount *= -1

    # initialize position after
    
    while not(switchLO.is_pressed or switchLI.is_pressed or switchRI.is_pressed or switchRO.is_pressed):
        step2.on()
        step2.off()
        rightcount +=1
        time.sleep(minTimeDelay)

    print("LO")
    print(switchLO.is_pressed)
    print("LI")
    print(switchLI.is_pressed)
    print("RO")
    print(switchRO.is_pressed)
    print("RI")
    print(switchRI.is_pressed)
 

    right_position = 0

    time.sleep(0.5)

    #fold right
    dir2.off()
    
    while (right_position < (rightcount - accStep)) and (not(switchLO.is_pressed or switchLI.is_pressed  or switchRO.is_pressed)):
        step2.on()
        step2.off()
        right_position +=1
        time.sleep(minTimeDelay)

    right_position = decelerateSingle(step2, dir2, switchLO, switchLI, switchRI, switchRO, accRevs, minTimeDelay, maxTimeDelay, right_position)



    left_position, right_position = centerX(step1, dir1, step2, dir2, switchLO, switchLI, switchRI, switchRO, left_position, right_position)

    x_mm, y_mm = get_CurrentLocation(left_position, right_position)

    time.sleep(0.5)

    return left_position, right_position, x_mm, y_mm

    

def centerX(step1, dir1, step2, dir2, switchLO, switchLI, switchRI, switchRO, left_motor, right_motor):

    center_steps = abs(left_motor - right_motor)/2

    print(center_steps)
    
    if left_motor > right_motor:

        #move right
        dir1.on()
        dir2.off()
        stepcount = 0

        while (stepcount < center_steps) and (not(switchLO.is_pressed or switchLI.is_pressed or switchRI.is_pressed or switchRO.is_pressed)):
            step1.on()
            step2.on()
            step1.off()
            step2.off()

            left_motor -=1
            right_motor +=1
            stepcount +=1
            
            time.sleep(0.0000030)
    

    elif right_motor > left_motor:

        #move left
        dir1.off()
        dir2.on()
        stepcount = 0

        while (stepcount < center_steps) and (not(switchLO.is_pressed or switchLI.is_pressed or switchRI.is_pressed or switchRO.is_pressed)):
            step1.on()
            step2.on()
            step1.off()
            step2.off()

            left_motor +=1
            right_motor -=1
            stepcount +=1
        
            time.sleep(0.000030)

    else:

        time.sleep(0.1)

    return left_motor, right_motor



def fold(step1, dir1, step2, dir2, switchLO, switchLI, switchRI, switchRO, left_motor, right_motor):

    first_Z = -25 # mm

    X1, Z1 = get_CurrentLocation(left_motor, right_motor)

    z_StepDiff = z_StepDifference(left_motor, right_motor, first_Z)

    count = 0

    if left_motor > right_motor:

        dir2.off()

        while (right_motor < left_motor) and (count < z_StepDiff) and (not(switchLO.is_pressed or switchLI.is_pressed or switchRI.is_pressed or switchRO.is_pressed)):
            step2.on()
            step2.off()

            time.sleep(0.0000030)

            right_motor += 1
            count +=1

    elif right_motor > left_motor:
        
        dir1.off()

        while (left_motor < right_motor) and (count < z_StepDiff) and (not(switchLO.is_pressed or switchLI.is_pressed or switchRI.is_pressed or switchRO.is_pressed)):
            step1.on()
            step1.off()

            time.sleep(0.0000030)

            left_motor += 1
            count +=1

    X2, Z2 = get_CurrentLocation(left_motor, right_motor)

    second_Z = first_Z - (Z1 - Z2)

    left_motor, right_motor = moveInZ(step1, dir1, step2, dir2, switchLO, switchLI, switchRI, switchRO, left_motor, right_motor, second_Z)

    left_motor, right_motor = centerX(step1, dir1, step2, dir2, switchLO, switchLI, switchRI, switchRO, left_motor, right_motor)

    foldedSteps = 27000 # hard coded position for folded -- move to prox switch in future

    dir1.off()
    dir2.off()

    while (left_motor < foldedSteps) and (right_motor < foldedSteps) and (not(switchLO.is_pressed or switchLI.is_pressed or switchRI.is_pressed or switchRO.is_pressed)):
        step1.on()
        step2.on()
        step1.off()
        step2.off()

        left_motor += 1
        right_motor += 1

        time.sleep(0.0000030)

    x_mm, z_mm = get_CurrentLocation(left_motor, right_motor)

    return left_motor, right_motor, x_mm, z_mm



def accelerateSingle(step1, dir1, switchLO, switchLI, switchRI, switchRO, revs, timeDelayMin, timeDelayMax, motor_steps):

    stepPerRev = 3200
    accSteps = stepPerRev * revs

    count = 0
    timeDelay = timeDelayMax

    stepup = (timeDelayMax-timeDelayMin)/accSteps

    direction = ((dir1.value / -1) + 1) - dir1.value # math logic to call direction -1 if dir1.on or +1 if dir1.off

    while (count < accSteps) and not(switchLO.is_pressed or switchLI.is_pressed or switchRI.is_pressed or switchRO.is_pressed):

        step1.on()
        step1.off()
        timeDelay -= stepup
        count +=1
        motor_steps += direction

        time.sleep(timeDelay)

    return motor_steps



def decelerateSingle(step1, dir1, switchLO, switchLI, switchRI, switchRO, revs, timeDelayMin, timeDelayMax, motor_steps):

    stepPerRev = 3200
    accSteps = stepPerRev * revs

    count = 0
    timeDelay = timeDelayMin

    stepup = (timeDelayMax-timeDelayMin)/accSteps

    direction = ((dir1.value / -1) + 1) - dir1.value # math logic to call direction -1 if dir1.on or +1 if dir1.off

    while (count < accSteps) and not(switchLO.is_pressed or switchLI.is_pressed or switchRI.is_pressed or switchRO.is_pressed):

        step1.on()
        step1.off()
        timeDelay += stepup
        count +=1
        motor_steps += direction

        time.sleep(timeDelay)

    return motor_steps



def moveInZ(step1, dir1, step2, dir2, switchLO, switchLI, switchRI, switchRO, left_motor, right_motor, desiredZ):

    x_mm, z_mm = get_CurrentLocation(left_motor, right_motor)

    z_future = z_mm + desiredZ
 
    step_left, step_right = inverseKinematics(x_mm, z_future) # desired steps at end of z movement

    if left_motor > step_left:
        dir1.on()
        dir2.on()

        while (left_motor > step_left) and (not(switchLO.is_pressed or switchLI.is_pressed or switchRI.is_pressed or switchRO.is_pressed)):
            step1.on()
            step2.on()
            step1.off()
            step2.off()
            left_motor -= 1
            right_motor -= 1

            time.sleep(0.0000030)

        return left_motor, right_motor

    else:
        dir1.off()
        dir2.off()

        while (left_motor < step_left) and (not(switchLO.is_pressed or switchLI.is_pressed or switchRI.is_pressed or switchRO.is_pressed)):
            step1.on()
            step2.on()
            step1.off()
            step2.off()
            left_motor += 1
            right_motor +=1

            time.sleep(0.0000030)

    x, z = get_CurrentLocation(left_motor, right_motor)

    print(x_mm, z_mm)

    return left_motor, right_motor


def moveInX(step1, dir1, step2, dir2, switchLO, switchLI, switchRI, switchRO, left_motor, right_motor, desiredX):

    x_mm, z_mm = get_CurrentLocation(left_motor, right_motor)

    x_future = x_mm + desiredX
 
    step_left, step_right = inverseKinematics(x_future, z_mm) # desired steps at end of z movement

    if left_motor > step_left:

        dir1.on()
        dir2.off()

        while (left_motor > step_left) and (not(switchLO.is_pressed or switchLI.is_pressed or switchRI.is_pressed or switchRO.is_pressed)):
            step1.on()
            step2.on()
            step1.off()
            step2.off()
            left_motor -= 1
            right_motor += 1

            time.sleep(0.0000030)

        return left_motor, right_motor

    else:
        dir1.off()
        dir2.on()

        while (left_motor < step_left) and (not(switchLO.is_pressed or switchLI.is_pressed or switchRI.is_pressed or switchRO.is_pressed)):
            step1.on()
            step2.on()
            step1.off()
            step2.off()
            left_motor += 1
            right_motor -= 1

            time.sleep(0.0000030)
    
    x_mm, z_mm = get_CurrentLocation(left_motor, right_motor)

    return left_motor, right_motor



def get_CurrentLocation(left_motor, right_motor): # get XZ position from current motor steps

    left_motor = float(left_motor*-1)
    right_motor = float(right_motor*1)
    
    stepRev = float(3200)
    mmRev = float(25)

    left_dist = (left_motor / stepRev) * mmRev #gives left dist
    right_dist = (right_motor / stepRev) * mmRev

    Length = float(350)  #linkage lengths
    EndEffect = float(65) # center of end effector to outer linkage pin center 

    linearStage = float(250); # travel of linear actuators
    OuterPinDist = float(56.5); # distance from edge of carriage mount to center of 8mm pin (x dist)
    ActuatorOffset = float(93.5 + 30); # edge of actuator (inside edge of aluminum bloc) to center of macron rail
    
    offset = float(ActuatorOffset + OuterPinDist); # center of macron rail to outer linkage pin center (fully ext)

    A = (-1*offset) + left_dist
    B = offset + right_dist
    
    centerX = (A + B) / 2
    Ax = (A*-1) + (centerX - EndEffect)

    Z_End = (math.sqrt((Length**2) - (Ax**2))) + 20

    return centerX, Z_End



def inverseKinematics(x_cord, z_cord): # get stepper positions from desired XZ cords

    z_cord -= 20
    
    Length = float(350)  #linkage lengths
    EndEffect = float(65) # center of end effector to outer linkage pin center 

    linearStage = float(250) # travel of linear actuators
    OuterPinDist = float(56.5) # distance from edge of carriage mount to center of 8mm pin (x dist)

    ActuatorOffset = float(93.5 + 30) # edge of actuator (inside edge of aluminum bloc) to center of macron rail
    Offset = float(ActuatorOffset + OuterPinDist); # center of macron rail to outer linkage pin center (fully ext)

    mmToStep = 3200.0 / 25.0

    Z_limit = float(50) #mm ; how close endeffector can get to macron (3 inches offset)
    Z_max = math.sqrt((Length**2) - ((Offset - EndEffect)**2))
    X_max_limit = float(Offset + linearStage); #mm ; how far can either (carriage mount outer pin) travel in x (fully fold)
    X_min_limit = float(Offset) #mm ; how close ...^ (from centerline of macron) (this is fully ext)

    theta = math.asin(z_cord/Length) # theta(left) = theta(right) due to linkage configuration
    AB = ((Length * math.cos(theta)) + EndEffect)*2; # distance from center of left carriage to center of right carriage

    OutSpace = X_max_limit - (AB/2) # distance from either carriage to outer (fully folded) boundary
    InSpace = (AB/2) - X_min_limit # disannce from either carriage to inner (fully ext) boundary
    Space = min(OutSpace, InSpace) # whichever boundary is closest is the limiting factor of x movement

    AB_max = float((EndEffect + math.sqrt((Length**2) - (Z_limit**2)))*2); # asssuming A = B; what is max value of A and B (position of linear stages)

    Ax = ((-1*AB/2) + Offset + x_cord) * -1 * mmToStep # left linear stage relative cord
    Bx = ((AB/2) - Offset + x_cord) * mmToStep # right linear stage relative cord

    if (z_cord < Z_limit) or (abs(x_cord) > Space):
        return;
    
    else:
        cords = [Ax, Bx]
        return cords



def z_StepDifference(left_motor, right_motor, deltaZ):

    Length = float(350)  #linkage lengths
    EndEffect = float(65) # center of end effector to outer linkage pin center 

    OuterPinDist = float(56.5); # distance from edge of carriage mount to center of 8mm pin (x dist)
    ActuatorOffset = float(93.5 + 30); # edge of actuator (inside edge of aluminum bloc) to center of macron rail

    curXZ = get_CurrentLocation(left_motor, right_motor)

    Z1 = curXZ[1]

    Z2 = Z1 + deltaZ

    tot_Offset = 2*(OuterPinDist+ActuatorOffset)

    left_mm = left_motor * 25.0 / 3200.0
    right_mm = right_motor * 25.0 / 3200.0

    AB1 = left_mm + right_mm + tot_Offset

    AB2 = (2 * (math.sqrt(((AB1/2.0) - EndEffect)**2) - ((Z2**2) - (Z1**2)))) - EndEffect

    stepDif = (AB2 - AB1) * 3200.0 / 25.0

    return stepDif
    

#define limit switches
sLO = Button(5, pull_up = True)
sLI = Button(1, pull_up = True)
sRI = Button(3, pull_up = True)
sRO = Button(2, pull_up = True)

s1 = OutputDevice(pin=17)
d1 = OutputDevice(pin=18)
s2 = OutputDevice(pin=4)
d2 = OutputDevice(pin=27)

### RUN ###
leftM, rightM, x, z = calibrateZero(s1, d1, s2, d2, sLO, sLI, sRI, sRO)

#leftM, rightM  = moveInZ(s1, d1, s2, d2, sLO, sLI, sRI, sRO, leftM, rightM, 50)

#leftM, rightM  = moveInX(s1, d1, s2, d2, sLO, sLI, sRI, sRO, leftM, rightM, 25)

#time.sleep(3)

#leftM, rightM, x, z  = fold(s1, d1, s2, d2, sLO, sLI, sRI, sRO, leftM, rightM)

#print("FINAL\n")
#print(x)
#print(z)
#center(s1, d1, s2, d2, positions(1), positions(2))

#d1.off()
#accelerateSingle(s1, d1, sLO, sLI, sRI, sRO, 1, 0.000030, 0.0003, 0)
#decelerateSingle(s1, d1, sLO, sLI, sRI, sRO, 1, 0.000030, 0.0003, 0)
#time.sleep(1)



