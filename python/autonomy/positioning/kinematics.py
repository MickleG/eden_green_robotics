import math;
import pyautogui;
import time;

def Kinematics(left_motor, right_motor): # get stepper positions from desired XZ cords

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


mmRev = float(25)
stepRev = float(3200)

rightMM = ((220) / (mmRev)) * stepRev
leftMM = ((220) / (mmRev)) * stepRev
    
cords = Kinematics(rightMM, leftMM)

print("\nX Position :   " + str(round(cords[0],4)))
print("Z Position :   " + str(round(cords[1],4)))


