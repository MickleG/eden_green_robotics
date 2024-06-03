#include <stdio.h> 
#include <stdlib.h>
#include <wiringPi.h>   // Include WiringPi library!
#include <time.h>       // For NANOS function
#include <iostream>
#include <cmath>        // For sqrt and other math functions

using namespace std;

class EndEffectorConfig 
{
    private:

        const float stepPerRev = 3200.0;
        const float mmPerRev = 25.0;
        const float linkLength = 350.0;
        const float carriageOffset = (93.5 + 30.0 + 56.5); // inside edge of aluminum bloc to center of macron rail + aluminum block thickness + carriage pin to inside edge of carriage
        const float endOffset = 130.0; // center of end effector to outer linkage pin center
        const float linearStage = 250.0; // mm of travel possible for linear actuators

        float maxAbs(float x, float y)
        {
            x = abs(x);
            y = abs(y);
            
            if(x > y) {return x;}

            else {return y;}
        }

        uint64_t nanos()
        {
            struct  timespec ts;

            clock_gettime (CLOCK_MONOTONIC_RAW, &ts);

            return (uint64_t)ts.tv_sec * (uint64_t)1000000000 + (uint64_t)(ts.tv_nsec);
        }


    public:

        MotorConfig leftMotor; // initialize left motor object
        MotorConfig rightMotor; // initialize right motor object

        float leftMotorPosition; // motor Position in mm
        float rightMotorPosition; // motor Position in mm
        
        float rightMotorSpeed; // speed in %
        float leftMotorSpeed; // speed in %

        float xPosition; // end effector position in mm (relative to center of macron)
        float zPosition; // end effeector position in mm

        float xDirection; // x component of direction vector (direction vector = vector of magnitude 1.0)
        float zDirection; // z component of direction vector (direction vector = vector of magnitude 1.0)

        float baseLength; // length between outer pins on carriages
        float basePartial; // for right triangle of hypotenuse link length
        
        float tempMax; // used for temporarily holding max value of rightMotorSpeed and leftMotorSpeed

        // INSTANCE MODIFIER

        EndEffectorConfig(int left, int right)
        {

            leftMotor.setHardware(17, 18, 5, 1);
            rightMotor.setHardware(4, 27, 2, 3); 

            leftMotor.setStepPosition(left);
            rightMotor.setStepPosition(right);
            
            leftMotorPosition = 0; // motor Position in mm
            rightMotorPosition = 0; // motor Position in mm
        
            rightMotorSpeed = 0; // *1.6 = mm/s
            leftMotorSpeed = 0; // *1.6 = mm/s

            baseLength = 0; // length between outer pins on carriages
            basePartial = 0; // for right triangle of hypotenuse link length

            xPosition = 0; // relative to center of macron
            zPosition = 0; // relative to motor plane

            xDirection = 0; // right relative to robot = (+)
            zDirection = 0; // forward relative to robot =(+)

        }

        void setGlobalAcceleration(float acc)
        {
            leftMotor.setAcceleration(acc);
            rightMotor.setAcceleration(acc);
        }



        // Position FUNCTIONS using IK and FK

        void updateMotorPosition() // converts left and right step counts to mm position (left = negative ; right = positive)
        {
            leftMotorPosition = (((float)leftMotor.stepCount) / stepPerRev) * mmPerRev; // mm
            rightMotorPosition = (((float)rightMotor.stepCount) / stepPerRev) * mmPerRev; // mm
        }

        void updateCurrentPosition() // get end effector XZ position from current motor steps (opposite of inverseKinematics)
        {

            updateMotorPosition();

            baseLength = leftMotorPosition + (carriageOffset * 2) + rightMotorPosition; // distance between left carriage and right carriage
            basePartial = (baseLength - endOffset) / 2.0; // right triangle component along linear actuator axis (linkage length is hyptenuse, z cord is height)

            zPosition = sqrt((linkLength * linkLength) - (basePartial * basePartial)) + 20; // current z cord of end effector (dist from outer pin to motor plane)
            xPosition = ((leftMotorPosition*-1) + rightMotorPosition) / 2; // current x cord of end effector relative to center plane of macron rail

        }

        void goToPosition(float xCord, float zCord, float speed)
        {
            updateMotorPosition();

            zCord = zCord - 20.0; // z direction offset -- Frame of reference changed from the motor plane to the outer pin z cordinate with this operation

            baseLength = (2 * sqrt((linkLength * linkLength) - (zCord * zCord))) + (endOffset); // DESIRED base length, not the current

            float goalRightMotorPosition = (((2 * xCord) + baseLength) / 2.0) - carriageOffset; // goal right position in mm ( to be used for next line )
            float goalLeftMotorPosition = (baseLength - goalRightMotorPosition - (2*carriageOffset)); // goal left position in mm
            
            if ((zCord < 50) || zCord > 330.6 || goalRightMotorPosition > 250.0 || goalLeftMotorPosition > 250.0)
            {
                printf("Error Goal Position out of Bounds\n");
            }

            else
            {
                
                float leftDelta = abs(leftMotorPosition - goalLeftMotorPosition); // find distance left motor needs to cover (mm)
                float rightDelta = abs(rightMotorPosition - goalRightMotorPosition); // find distance right motor needs to cover (mm)
                
                leftMotorSpeed = speed * ((leftMotorPosition - goalLeftMotorPosition) / leftDelta); // change direction of motor towards goal position (this could be left out)
                rightMotorSpeed = speed * ((rightMotorPosition - goalRightMotorPosition) / rightDelta);
                
                if(leftDelta - rightDelta > 0)
                {
                    if(((rightDelta / leftDelta) * speed) >= 1) // avoid motor deadband
                    {
                        rightMotorSpeed = (rightDelta / leftDelta) * rightMotorSpeed; // partial speed to get there at same time as left motor
                    }
                }
                
                else if(leftDelta - rightDelta < 0)
                {
                    if(((leftDelta / rightDelta) * speed) >= 1) // avoid motor deadband
                    {
                        leftMotorSpeed = (leftDelta / rightDelta) * leftMotorSpeed; // partial speed to get there at same time as right motor
                    }
                }
                
                goalLeftMotorPosition = goalLeftMotorPosition * (stepPerRev / mmPerRev);
                goalRightMotorPosition = goalRightMotorPosition * (stepPerRev / mmPerRev);
                
                printf("\n\nLEFT speed: %f %\n", leftMotorSpeed);
                printf("\nRIGHT speed: %f %\n", rightMotorSpeed);
                
                leftMotor.setGoalPosition(goalLeftMotorPosition);
                rightMotor.setGoalPosition(goalRightMotorPosition);

                while(rightMotor.findingTarget || leftMotor.findingTarget) 
                {
                    findingLeft = leftMotor.goalPositionACC(); // TEST WITH ACC
                    findingRight = rightMotor.goalPositionACC(); // TEST WITH ACC
                }
            }

        }

        void calibrateZero(float speed) // NEEDS UPDATED FOR ACCELERATION
        {
            rightMotor.setSpeed(0);
            leftMotor.setSpeed(0);

            while (digitalRead(rightMotor.limitOutside)) // check right switch
            {

                rightMotor.accToSpeed(speed*-1); // move right relative to robot
                leftMotor.accToSpeed(speed); // move right relative to robot

                leftMotor.motorDrive();
                rightMotor.motorDrive();

                //leftMotor.controlLoop(); // probably safer to use this even tho not needed
                //rightMotor.controlLoop();
            }

            rightMotor.setStepPosition(32000-256); // because the switch usually hits about 2 mm from actually touching end


            while (digitalRead(leftMotor.limitOutside))
            {
                leftMotor.accToSpeed(speed*-1);
                rightMotor.accToSpeed(speed);

                leftMotor.motorDrive();
                rightMotor.motorDrive();
            }

            leftMotor.setStepPosition(32000-256);
            
            updateCurrentPosition();

            goToPosition(0, 100, 30);
        }


        // Velocity Functions using method of kinematic coefficients + IK //

        void moveInX(float speed)
        {
            if(leftMotor.currentSpeed != speed || rightMotor.currentSpeed != (speed*=-1))
            {
                leftMotor.setSpeed(speed);
                rightMotor.setSpeed(speed * -1);
            }

            leftMotor.controlLoop();
            rightMotor.controlLoop(); 

        }
        

        void moveInZ(float speed)
        {
            if(leftMotor.currentSpeed != speed || rightMotor.currentSpeed != speed)
            {
                leftMotor.setSpeed(speed);
                rightMotor.setSpeed(speed);
            }

            leftMotor.controlLoop(); 
            rightMotor.controlLoop(); 

        }


        void directionalDrive(float CMD_X_DIST, float CMD_Z_DIST, float speed) // drives at set direction and speed (needs to be actively updating every step)
        {
            updateMotorPosition();

            baseLength = (leftMotorPosition) + (carriageOffset * 2) + rightMotorPosition; // distance between left carriage and right carriage
            basePartial = (baseLength - endOffset) / 2.0; // right triangle component along linear actuator axis (linkage length is hyptenuse, z cord is height)
            
            rightMotorSpeed = (CMD_X_DIST) - (CMD_Z_DIST * (sqrt((linkLength * linkLength) - (basePartial * basePartial)) / basePartial)); // where CMDXDIR = dx/dt and CMDZDIR = dz/dt > finding dRM/dt (this value * 1.6 = mm/s)
            leftMotorSpeed = ((2 * CMD_X_DIST) - rightMotorSpeed); // using dRM/dt to get dLM/dt (this value * 1.6 = mm/s)
            
            tempMax = maxAbs(rightMotorSpeed, leftMotorSpeed); // find the motor with greater speed magnitude
            rightMotorSpeed = (rightMotorSpeed / tempMax) * speed * -1; // convert motor speed to speed commanded by function input argument "speed" (switch signs)
            leftMotorSpeed = (leftMotorSpeed / tempMax) * speed; // negative to keep frame of reference consistent with right motor (keep sign)

            leftMotor.setSpeed(leftMotorSpeed); // declaring left motor speed
            rightMotor.setSpeed(rightMotorSpeed); // declaring right motor speed

            leftMotor.controlLoop(); // activates motor control loop > drives with setSpeedCommands while watching limits
            rightMotor.controlLoop(); // activates motor control loop > drives with setSpeedCommands while watching limits

        }

};