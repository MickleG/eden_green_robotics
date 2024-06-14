// include header file for this Class

#include "EndEffectorConfig.h"
#include <stdint.h>     // For uint8_t, uint16_t, uint32_t, uint64_t
#include <stdio.h>      // For printf statements
#include <stdlib.h>
#include <iostream>
#include <time.h>       // For NANOS function
#include <cmath>        // For sqrt and other math functions
#include "MotorConfig.h"// For MotorConfig class and functions

using namespace std;

//class EndEffectorConfig

    //private

    float EndEffectorConfig::maxAbs(float x, float y)
    {
        x = abs(x);
        y = abs(y);
        
        if(x > y) {return x;}

        else {return y;}
    }

    uint64_t EndEffectorConfig::nanos()
    {
        struct  timespec ts;

        clock_gettime (CLOCK_MONOTONIC_RAW, &ts);

        return (uint64_t)ts.tv_sec * (uint64_t)1000000000 + (uint64_t)(ts.tv_nsec);
    }


    //public

    // INSTANCE MODIFIER

    EndEffectorConfig::EndEffectorConfig(int left, int right)
    {

        leftMotor.setHardware(17, 18, 5, 1);
        rightMotor.setHardware(4, 27, 2, 3);
        yMotor.setHardware(25, 24, 21, 13); // defines the y stage motor, up is positive speed


        leftMotor.setStepPosition(left);
        rightMotor.setStepPosition(right);
        yMotor.setStepPosition(0);
        
        leftMotorPosition = 0; // motor Position in mm
        rightMotorPosition = 0; // motor Position in mm
        yMotorPosition = 0;
    
        rightMotorSpeed = 0; // *1.6 = mm/s
        leftMotorSpeed = 0; // *1.6 = mm/s
        yMotorSpeed = 0;

        //baseLength = 0; // length between outer pins on carriages
        //basePartial = 0; // for right triangle of hypotenuse link length

        xPosition = 0; // relative to center of macron
        yPosition = 0; // relative to bottom of macron
        zPosition = 0; // relative to motor plane

        //xDirection = 0; // right relative to robot = (+)
        //zDirection = 0; // forward relative to robot = (+)

        calibrationSuccess = 0; // initialize robot to require calibration

    }

    void EndEffectorConfig::setGlobalAcceleration(float acc)
    {
        leftMotor.setAcceleration(acc);
        rightMotor.setAcceleration(acc);
    }

    // Position FUNCTIONS using IK and FK

    void EndEffectorConfig::updateMotorPosition() // converts left and right step counts to mm position (left = negative ; right = positive)
    {
        leftMotorPosition = (((float)leftMotor.stepCount) / stepPerRev) * mmPerRev; // mm
        rightMotorPosition = (((float)rightMotor.stepCount) / stepPerRev) * mmPerRev; // mm
    }

    void EndEffectorConfig::updateCurrentPosition() // get end effector XZ position from current motor steps (opposite of inverseKinematics)
    {

        updateMotorPosition();

        float baseLength = leftMotorPosition + (carriageOffset * 2) + rightMotorPosition; // distance between left carriage and right carriage
        float basePartial = (baseLength - endOffset) * 0.5; // right triangle component along linear actuator axis (linkage length is hyptenuse, z cord is height)

        zPosition = sqrt((linkLength * linkLength) - (basePartial * basePartial)) + 20; // current z cord of end effector (dist from outer pin to motor plane)
        xPosition = ((leftMotorPosition*-1) + rightMotorPosition) / 2; // current x cord of end effector relative to center plane of macron rail
        yPosition = yMotor.stepCount * revPerStep * mmPerRevY;
    }

    void EndEffectorConfig::goToPosition(float xCord, float zCord, float speed)
    {
        updateMotorPosition();


        zCord = zCord - 20.0; // z direction offset -- Frame of reference changed from the motor plane to the outer pin z cordinate with this operation

        float baseLength = (2 * sqrt((linkLength * linkLength) - (zCord * zCord))) + (endOffset); // DESIRED base length, not the current

        float goalRightMotorPosition = (((2 * xCord) + baseLength) * 0.5) - carriageOffset; // goal right position in mm ( to be used for next line )
        float goalLeftMotorPosition = (baseLength - goalRightMotorPosition - (2*carriageOffset)); // goal left position in mm
        
        if ((zCord < 50) || zCord > 330.6 || goalRightMotorPosition > 250.0 || goalLeftMotorPosition > 250.0)
        {
            printf("Error Goal Position out of Bounds\n"); // if its impossible to go to that position
        }

        else
        {
            
            float leftDelta = abs(leftMotorPosition - goalLeftMotorPosition); // find distance left motor needs to cover (mm)
            float rightDelta = abs(rightMotorPosition - goalRightMotorPosition); // find distance right motor needs to cover (mm)
            
            leftMotorSpeed = speed * ((leftMotorPosition - goalLeftMotorPosition) / leftDelta); // change left motor direction and spweed towards goal position
            rightMotorSpeed = speed * ((rightMotorPosition - goalRightMotorPosition) / rightDelta); // change right motor direction and spweed towards goal position
            
            // when left motor has farther to travel
            if(leftDelta - rightDelta > 0)
            {
                if(((rightDelta / leftDelta) * speed) >= 1) // avoid motor deadband
                {
                    rightMotorSpeed = (rightDelta / leftDelta) * rightMotorSpeed; // partial speed to get there at same time as left motor
                }
            }
            
            // when right motor has farther to travel
            else if(leftDelta - rightDelta < 0)
            {
                if(((leftDelta / rightDelta) * speed) >= 1) // avoid motor deadband
                {
                    leftMotorSpeed = (leftDelta / rightDelta) * leftMotorSpeed; // partial speed to get there at same time as right motor
                }
            }
            
            goalLeftMotorPosition = goalLeftMotorPosition * (stepPerRev / mmPerRev); // convert from mm to steps 
            goalRightMotorPosition = goalRightMotorPosition * (stepPerRev / mmPerRev); // convert from mm to steps
            
            leftMotor.setGoalPosition(goalLeftMotorPosition, abs(leftMotorSpeed)); // use this function to set characteristics of goal position movement
            rightMotor.setGoalPosition(goalRightMotorPosition, abs(rightMotorSpeed));

            while(rightMotor.findingTarget || leftMotor.findingTarget)
            {
                leftMotor.goalPosition(); // always use goalPosition() in a loop, as "controlLoop" is not in a while loop in goalPosition() and is called based on condition
                rightMotor.goalPosition();
                updateCurrentPosition();
                // cout << "currentPosition: " << xPosition << ", " << yPosition << ", " << zPosition << endl;
                motorMoving = true;
            }
        }

        updateCurrentPosition();
        motorMoving = false;

    }


    void EndEffectorConfig::calibrateZero(float speed) // NEEDS UPDATED FOR ACCELERATION
    {
        rightMotor.setSpeed(0);
        leftMotor.setSpeed(0);
 
        // ping outer right switch
        while (leftMotor.driveState == 1 && rightMotor.driveState == 1) // monitor unitl limit switch is pressed
        {
            rightMotor.setSpeed(speed*-1); // set right motor speed to drive right
            leftMotor.setSpeed(speed); // set left motor speed to drive right

            rightMotor.controlLoop(); // run right motor
            leftMotor.controlLoop(); // run left motor
        }

        if(rightMotor.driveState == -1)
        {
            rightMotor.setStepPosition(32000-256); // because the switch usually hits about 2 mm from actually touching end

             // ping outer left switch
            while (leftMotor.driveState == 1 && rightMotor.driveState != 2) // monitor until limit switch is pressed
            {
                leftMotor.setSpeed(speed*-1); // set left motor to drive left
                rightMotor.setSpeed(speed); // set right motor to drive left

                leftMotor.controlLoop(); // run left motor
                rightMotor.controlLoop(); // run right motor
            }

            if(leftMotor.driveState == -1)
            {
                leftMotor.setStepPosition(32000-256);
                calibrationSuccess = 1;

                goToPosition(0, 100, 100);
            }

            else { calibrationSuccess = 0; }
        }

        if(leftMotor.driveState == 2)
        {
            leftMotor.setStepPosition(0+256); // because the switch usually hits about 2 mm from actually touching end

             // ping inner right switch
            while (rightMotor.driveState == 1 && leftMotor.driveState != -1) // monitor until limit switch is pressed
            {
                leftMotor.setSpeed(speed*-1); // set left motor to drive left
                rightMotor.setSpeed(speed); // set right motor to drive left

                leftMotor.controlLoop(); // run left motor
                rightMotor.controlLoop(); // run right motor
            }

            if(rightMotor.driveState == 2)
            {
                rightMotor.setStepPosition(0+256);
                calibrationSuccess = 1;

                goToPosition(0, 100, 100);
            }

            else { calibrationSuccess = 0; }
        }

        
    }


    // Velocity Functions using method of kinematic coefficients + IK //

    void EndEffectorConfig::moveInX(float speed)
    {
        if(leftMotor.currentSpeed != speed || rightMotor.currentSpeed != (speed*=-1))
        {
            leftMotor.setSpeed(speed);
            rightMotor.setSpeed(speed * -1);
        }

        leftMotor.controlLoop();
        rightMotor.controlLoop(); 

    }
    

    void EndEffectorConfig::moveInZ(float speed)
    {
        if(leftMotor.currentSpeed != speed || rightMotor.currentSpeed != speed)
        {
            leftMotor.setSpeed(speed);
            rightMotor.setSpeed(speed);
        }

        leftMotor.controlLoop(); 
        rightMotor.controlLoop(); 

    }

    void EndEffectorConfig::moveInZAccel(float speed, float acceleration)
    {
        setGlobalAcceleration(acceleration);

        if(leftMotor.currentSpeed != speed || rightMotor.currentSpeed != speed)
        {
            leftMotor.accToSpeed(speed);
            rightMotor.accToSpeed(speed);
        }

        leftMotor.controlLoop(); 
        rightMotor.controlLoop(); 

    }


    void EndEffectorConfig::directionalDrive(float CMD_X_SPEED, float CMD_Z_SPEED) // drives at set direction and speed (needs to be actively updating every step)
    {

        updateMotorPosition();

        float baseLength = (leftMotorPosition) + (carriageOffset * 2) + rightMotorPosition; // distance between left carriage and right carriage
        float basePartial = (baseLength - endOffset) * 0.5; // right triangle component along linear actuator axis (linkage length is hyptenuse, z cord is height)
        float tanTheta = sqrt((linkLength * linkLength) - (basePartial * basePartial)) / basePartial; // 
        float MAX_Z_SPEED = (MOTOR_SPEED_MAX - MAX_X_SPEED) / tanTheta; // highest commanded z value allowed for current position

        if((abs(CMD_X_SPEED) <= MAX_X_SPEED) && (CMD_Z_SPEED <= MAX_Z_SPEED))
        {
            rightMotorSpeed = (CMD_X_SPEED) - (CMD_Z_SPEED * tanTheta); // where CMDXDIR = dx/dt and CMDZDIR = dz/dt > finding dRM/dt (this value * 1.6 = mm/s)
            leftMotorSpeed = ((2 * CMD_X_SPEED) - rightMotorSpeed); // using dRM/dt to get dLM/dt (this value * 1.6 = mm/s)

            leftMotor.setSpeed(leftMotorSpeed); // declaring left motor speed
            rightMotor.setSpeed(rightMotorSpeed); // declaring right motor speed
            
            leftMotor.controlLoop(); // run the left motor
            rightMotor.controlLoop(); // run the right motor
        }

        else
        {
            if(abs(CMD_X_SPEED) > MAX_X_SPEED)
            {
                printf("\nCommanded X Speed: %f >> Out of Bounds\n", CMD_X_SPEED);
            }

            if(abs(CMD_Z_SPEED) > MAX_Z_SPEED)
            {
                printf("\nCommanded Z Speed: %f >> Out of Bounds\n", CMD_Z_SPEED);
            }
        }

    }

    void EndEffectorConfig::directionalDrive_MAG(float CMD_X_HEADING, float CMD_Z_HEADING, float speed) // drives at set direction and speed (needs to be actively updating every step)
    {
        updateMotorPosition();

        float baseLength = (leftMotorPosition) + (carriageOffset * 2) + rightMotorPosition; // distance between left carriage and right carriage
        float basePartial = (baseLength - endOffset) * 0.5; // right triangle component along linear actuator axis (linkage length is hyptenuse, z cord is height)
        
        rightMotorSpeed = (CMD_X_HEADING) - (CMD_Z_HEADING * (sqrt((linkLength * linkLength) - (basePartial * basePartial)) / basePartial)); // where CMDXDIR = dx/dt and CMDZDIR = dz/dt > finding dRM/dt (this value * 1.6 = mm/s)
        leftMotorSpeed = ((2 * CMD_X_HEADING) - rightMotorSpeed); // using dRM/dt to get dLM/dt (this value * 1.6 = mm/s)
        
        float tempMax = maxAbs(rightMotorSpeed, leftMotorSpeed); // find the motor with greater speed magnitude
        
        rightMotorSpeed = (rightMotorSpeed / tempMax) * speed * -1; // convert motor speed to speed commanded by function input argument "speed" (switch signs)
        leftMotorSpeed = (leftMotorSpeed / tempMax) * speed; // negative to keep frame of reference consistent with right motor (keep sign)

        leftMotor.setSpeed(leftMotorSpeed); // declaring left motor speed
        rightMotor.setSpeed(rightMotorSpeed); // declaring right motor speed
        
        leftMotor.controlLoop();
        rightMotor.controlLoop();

    }

