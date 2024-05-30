#include <stdio.h> 
#include <stdlib.h>
#include <wiringPi.h>   // Include WiringPi library!
#include <time.h>       // For NANOS function
#include <iostream>
#include <cmath>        // For sqrt and other math functions

#include <MOTORCONFIG.h>

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

        float MAX_X_SPEED = 25.0; // mm/s -- during the directionalDrive() function
        float MAX_Z_SPEED = 100; // mm/s -- during the directionalDrive() function


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
        bool calibrationSuccess = 0;

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
            zDirection = 0; // forward relative to robot = (+)

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
            basePartial = (baseLength - endOffset) * 0.5; // right triangle component along linear actuator axis (linkage length is hyptenuse, z cord is height)

            zPosition = sqrt((linkLength * linkLength) - (basePartial * basePartial)) + 20; // current z cord of end effector (dist from outer pin to motor plane)
            xPosition = ((leftMotorPosition*-1) + rightMotorPosition) / 2; // current x cord of end effector relative to center plane of macron rail

        }

        void goToPosition(float xCord, float zCord, float speed)
        {
            updateMotorPosition();

            zCord = zCord - 20.0; // z direction offset -- Frame of reference changed from the motor plane to the outer pin z cordinate with this operation

            baseLength = (2 * sqrt((linkLength * linkLength) - (zCord * zCord))) + (endOffset); // DESIRED base length, not the current

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
                }
            }

            updateCurrentPosition();

        }


        void calibrateZero(float speed) // NEEDS UPDATED FOR ACCELERATION
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
                calibrationSuccess = 1;
            }

            // ping outer left switch
            while (leftMotor.driveState == 1 && rightMotor.driveState == 1) // monitor until limit switch is pressed
            {
                leftMotor.setSpeed(speed*-1); // set left motor to drive left
                rightMotor.setSpeed(speed); // set right motor to drive left

                leftMotor.controlLoop(); // run left motor
                rightMotor.controlLoop(); // run right motor
            }

            if(leftMotor.driveState == -1 && calibrationSuccess)
            {
                leftMotor.setStepPosition(32000-256);
                calibrationSuccess = 1;

                goToPosition(0, 100, 50);
            }

            else { calibrationSuccess = 0; }
            
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


        void directionalDrive(float CMD_X_SPEED, float CMD_Z_SPEED) // drives at set direction and speed (needs to be actively updating every step)
        {
            if((abs(CMD_X_SPEED) <= MAX_X_SPEED) && (abs(CMD_Z_SPEED) <= MAX_Z_SPEED))
            {
                updateMotorPosition();

                baseLength = (leftMotorPosition) + (carriageOffset * 2) + rightMotorPosition; // distance between left carriage and right carriage
                basePartial = (baseLength - endOffset) * 0.5; // right triangle component along linear actuator axis (linkage length is hyptenuse, z cord is height)
                
                rightMotorSpeed = (CMD_X_SPEED) - (CMD_Z_SPEED * (sqrt((linkLength * linkLength) - (basePartial * basePartial)) / basePartial)); // where CMDXDIR = dx/dt and CMDZDIR = dz/dt > finding dRM/dt (this value * 1.6 = mm/s)
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

        void directionalDrive_MAG(float CMD_X_HEADING, float CMD_Z_HEADING, float speed) // drives at set direction and speed (needs to be actively updating every step)
        {
            updateMotorPosition();

            baseLength = (leftMotorPosition) + (carriageOffset * 2) + rightMotorPosition; // distance between left carriage and right carriage
            basePartial = (baseLength - endOffset) * 0.5; // right triangle component along linear actuator axis (linkage length is hyptenuse, z cord is height)
            
            rightMotorSpeed = (CMD_X_HEADING) - (CMD_Z_HEADING * (sqrt((linkLength * linkLength) - (basePartial * basePartial)) / basePartial)); // where CMDXDIR = dx/dt and CMDZDIR = dz/dt > finding dRM/dt (this value * 1.6 = mm/s)
            leftMotorSpeed = ((2 * CMD_X_HEADING) - rightMotorSpeed); // using dRM/dt to get dLM/dt (this value * 1.6 = mm/s)
            
            tempMax = maxAbs(rightMotorSpeed, leftMotorSpeed); // find the motor with greater speed magnitude
            rightMotorSpeed = (rightMotorSpeed / tempMax) * speed * -1; // convert motor speed to speed commanded by function input argument "speed" (switch signs)
            leftMotorSpeed = (leftMotorSpeed / tempMax) * speed; // negative to keep frame of reference consistent with right motor (keep sign)

            leftMotor.setSpeed(leftMotorSpeed); // declaring left motor speed
            rightMotor.setSpeed(rightMotorSpeed); // declaring right motor speed
            
            leftMotor.controlLoop();
            rightMotor.controlLoop();

        }

};
