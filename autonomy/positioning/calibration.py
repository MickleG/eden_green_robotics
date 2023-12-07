import RPi.GPIO as GPIO
import time


def positionCalibrate(lmp, lmd, rmp, rmd): # left motor pulse, left motor dir...
    
    switch1 = 2 #gpio 2
    switch2 = 5 #gpio 5
    switch3 = 3 #gpio 3
    switch4 = 6 #gpio 6
    #switch5 = 12 #gpio12 ***** REASSIGN THIS VALUE ******

    GPIO.setmode(GPIO.BCM)
    
    GPIO.setup(switch1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) #left actuator outer
    GPIO.setup(switch2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) #left actuator inner
    GPIO.setup(switch3, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) #right actuator inner
    GPIO.setup(switch4, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) #right actuator outer
    #GPIO.setup(switch5, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) #right actuator outer

    GPIO.setup(lmp, GPIO.OUT) #left motor pulse
    GPIO.setup(lmd, GPIO.OUT) #left motor dir
    GPIO.setup(rmp, GPIO.OUT) #right motor pulse
    GPIO.setup(rmd, GPIO.OUT) #left motor dir

    delay = 0.0000030 # delay between steps in seconds
    
    microStep = 3200.0 # number of steps per revolution
    mmPerRev = 12.5 # based on pitch of threaded stainless steel rod (IGUS)
    foldSteps = microStep * (220.0/mmPerRev) # folded position steps from calibration



    ### CALIBRATION SEQUENCE LEFT ### 

    GPIO.output(lmd, GPIO.HIGH) # set direction to move right
    stepsL = 0 # initialize steps
    
    while (GPIO.input(switch2)): # move right carriage extended

        GPIO.output(lmp, GPIO.HIGH)
        #time.sleep(delay)
        GPIO.output(lmp, GPIO.LOW)
        time.sleep(delay)
        
        stepsL +=1

    LC_Pos = 0 # set left position to 0
    GPIO.output(lmd, GPIO.LOW) # reverse direction



    ### CALIBRATION SEQUENCE RIGHT ###

    GPIO.output(lmd, GPIO.HIGH) # set direction to move left
    stepsR = 0 # initialize steps
    
    while not(GPIO.input(switch3)): # move right carriage extended

        GPIO.output(rmp, GPIO.HIGH)
        time.sleep(delay)
        GPIO.output(rmp, GPIO.LOW)
        time.sleep(delay)
        
        stepsR +=1


    RC_Pos = 0 # set right position to 0
    GPIO.output(rmd, GPIO.LOW) # reverse direction

    time.sleep(1)





    ### Fold SEQUENCE ### >> uses inductance based switch to find folding limit

    returnSteps = 0

    #while not(GPIO.input(switch5)) and (returnSteps < foldSteps) : # move carriages towards folded
        
        #GPIO.output(rmp, GPIO.HIGH)
        #GPIO.output(lmp, GPIO.HIGH)
        #GPIO.output(rmp, GPIO.LOW)
        #GPIO.output(lmp, GPIO.LOW)
        #time.sleep(delay)

        #returnSteps +=1

    #print('Calibration Complete')

    #return returnSteps

positionCalibrate(4, 27, 17, 18)

