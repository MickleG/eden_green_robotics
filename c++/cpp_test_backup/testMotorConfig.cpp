#include <stdio.h> 
#include <stdlib.h>
#include <wiringPi.h>   // Include WiringPi library!
#include <time.h>       // For NANOS function
#include <iostream>
#include <cmath>        // For sqrt and other math functions

using namespace std;

// to be created for left and right motor inside the endEffector Object
// commands for different motors instances can be used in the same while loop because we use relative time rather than delays to decide pulse timing (no need to multithread)

class MotorConfig
{   

    // private data members specific to our configuration -- dont want to ever modify
    private: 

        // internal functions

        uint64_t nanos()
        {
            struct  timespec ts;

            clock_gettime (CLOCK_MONOTONIC_RAW, &ts);

            return (uint64_t)ts.tv_sec * (uint64_t)1000000000 + (uint64_t)(ts.tv_nsec);
        }


    // public member function    
    public:

        //** RPI hardware configuration **//

        uint8_t stepPin; // needs to be pulsed between HIGH and LOW to achieve motion
        uint8_t dirPin; // HIGH = Drives INWARD (towards stepper motor) || LOW = Drives OUTWARD (away from stepper motor)

        uint8_t limitInside; // needs to be pulsed between HIGH and LOW to achieve motion
        uint8_t limitOutside; // HIGH = Drives INWARD (towards stepper motor) || LOW = Drives OUTWARD (away from stepper motor)


        //** Time Step for accurate pulsing of stepper motors in nanoseconds **//

        uint64_t currentTimeStep; // nanoseconds -- logged at the beggining of motor control loop function
        uint64_t prevTimeStep; // nanoseconds -- logged immediately after a pulse has happened
        uint64_t limTimeStep; // nanoseconds -- logged immediately after limit switch is triggered, used for debounce time


        //** Current Motor Characteristics **//

        bool phase; // phase of stepper motor (High 1 or Low 0) to power coils during actuation

        uint32_t currentDelay; // nanoseconds -- this indicates the speed of stepper motor and is updated with setSpeed funciton  (must be between Max and MinDelay)
        uint16_t stepCount; // microsteps -- configured automatically during calibration by using limit switch as physical reference, updated every time step is pulsed 
        int8_t motorDir; // 1 for inward velocity  ||  -1 for outward velocity
        int8_t driveState; // 0 = limited | 1 = drive | 2 = inner drive | 3 = outer drive
        
        float currentSpeed;


        //** GOAL (or desired) characterisitics for goal functions **//

        float desSpeed; // -100 to 100 set by "goalDrive()"
        float acceleration; // in mm / s^2


        // INSTANCE MODIFIERS // 

        MotorConfig(char Side)
        {
            wiringPiSetupGpio(); // Initialize wiringPi -- using Broadcom pin numbers
            
            if(Side == 'L')
            {
                stepPin = 17;
                dirPin = 18;
                limitInside = 1;
                limitOutside = 5;
            }

            if(Side == 'R')
            {
                stepPin = 4;
                dirPin = 27;
                limitInside = 3;
                limitOutside = 2;
            }
            
            pinMode(limitOutside, INPUT);      	
            pullUpDnControl(limitOutside, PUD_UP); 
    
            pinMode(limitInside, INPUT);      	   
            pullUpDnControl(limitInside, PUD_UP); 
            
            pinMode(stepPin, OUTPUT);
            pinMode(dirPin, OUTPUT);

            prevTimeStep = nanos(); // sets initial pulse time instance
            limTimeStep = nanos(); // sets initial limit trigger time instance
            currentTimeStep = nanos();

            phase = 0;
            driveState = 1;
            stepCount = 0; 

            setSpeed(0);

        }

        MotorConfig(uint8_t s, uint8_t d, uint8_t limOut, uint8_t limIn)
        {
            wiringPiSetupGpio(); // Initialize wiringPi -- using Broadcom pin numbers

            stepPin = s; // set new hardware gpio pin for motor step pin
            dirPin = d; // set new hardware gpio pin for motor direciton 

            limitOutside = limOut;
            limitInside = limIn;
            
            pinMode(limitOutside, INPUT);      	
            pullUpDnControl(limitOutside, PUD_UP); 
    
            pinMode(limitInside, INPUT);      	   
            pullUpDnControl(limitInside, PUD_UP); 
            
            pinMode(stepPin, OUTPUT);
            pinMode(dirPin, OUTPUT);

            prevTimeStep = nanos(); // sets initial pulse time instance
            limTimeStep = nanos(); // sets initial limit trigger time instance 
            currentTimeStep = nanos();
            
            phase = 0; // initialize stepper motor phase
            driveState = 1; // initialize driveState to normal (should be initialized to 1 after calibration) 
            stepCount = 0;

            setSpeed(0);
        }

        MotorConfig()
        {
            prevTimeStep = nanos(); // sets initial pulse time instance
            limTimeStep = nanos(); // sets initial limit trigger time instance 
            currentTimeStep = nanos();

            phase = 0; // initialize stepper motor phase
            driveState = 1; // initialize driveState to normal (should be initialized to 1 after calibration) 
            stepCount = 0;

            setSpeed(0); // ensuring setSpeed is at 0 so motors dont run when control loop starts
        }



        // SET FUNCTIONS //

        void setHardware(uint8_t s, uint8_t d, uint8_t limOut, uint8_t limIn)
        {
            wiringPiSetupGpio(); // Initialize wiringPi -- using Broadcom pin numbers

            stepPin = s; // set new hardware gpio pin for motor step pin
            dirPin = d; // set new hardware gpio pin for motor direciton 

            limitOutside = limOut; 
            limitInside = limIn;
            
            pinMode(limitOutside, INPUT);       
            pullUpDnControl(limitOutside, PUD_UP); 
    
            pinMode(limitInside, INPUT);           
            pullUpDnControl(limitInside, PUD_UP); 
            
            pinMode(stepPin, OUTPUT);
            pinMode(dirPin, OUTPUT);
        }
        
        void setStepPosition(uint16_t steps)
        {
            stepCount = steps;
        }


        void setPulseDelay(uint32_t delay) // directly change current pulse delay in ns
        {
            currentDelay = delay; // ns
        }


        // This function should be run as a subscriber to motor % commands from the jetson NANO

        void setSpeed(float speed) // use value from -100 to 100 to represent % speed -- deadband -10% to +10% -- see motorConfig.pdf for speed charts
        {
            currentSpeed = speed;
            
            if(speed >= 1 && speed <= 100)
            {
                digitalWrite(dirPin, 1); // if speed is positive, then drive inward
                currentDelay = (uint32_t)(4000000.0 / (1.6*speed)); // translate speed command to step delay in ns
                motorDir = -1; // inwards
            }

            else if(speed <= -1 && speed >= -100)
            {
                digitalWrite(dirPin, 0); // if speed is negative, then drive outward
                currentDelay = (uint32_t)(4000000.0 / (-1.6*speed)); // translate speed command to step delay in ns
                motorDir = 1; // outwards
            }

            else
            {
                currentDelay = 0;
            }

        }

        void setAcceleration(float acc)
        {
            acceleration = acc;
        }



        // MOTOR CONTROL FUNCTIONS //

        void moveIn()
        {
            digitalWrite(dirPin, 1);

            if(nanos() - prevTimeStep > currentDelay)
            {
                digitalWrite(stepPin, phase);
                prevTimeStep = nanos();
                stepCount += (1*phase);
                phase = !phase;
            }
        }

        void moveOut()
        {
            digitalWrite(dirPin, 0);

            if(nanos() - prevTimeStep > currentDelay)
            {
                digitalWrite(stepPin, phase);
                prevTimeStep = nanos();
                stepCount -= (1*phase);
                phase = !phase;
            }
        }

        // TO BE USED IN CONTROL LOOP
        void motorDrive()
        {
            currentTimeStep =  nanos();

            if(((currentTimeStep - prevTimeStep) > currentDelay) && (currentDelay > 0))
            {
                //printf("\nPhase: %d ns\n", currentTimeStep - prevTimeStep);
                digitalWrite(stepPin, phase);
                prevTimeStep = nanos();
                stepCount += (motorDir*phase);
                phase = !phase;
            }

        }

        // TO BE USED IN ROS Motor Control NODE -- updating in range 500MHz->2GHz
        void controlLoop()
        {
            currentTimeStep = nanos(); // test if this works better, or nanos() inline for each case 
            
            switch (driveState)
            {
                case 1:

                    motorDrive();
                    driveState = ((2 * digitalRead(limitOutside)) - digitalRead(limitInside)); // if both are 1 (not pressed) driveState = 1
                    //printf("DRIVE ENABLED.\n");

                    break;


                case -1: // if limitOutside is pressed

                    if (!digitalRead(limitOutside)) // if limitOutside remains pressed
                    {
                        if (motorDir < 0) {motorDrive();} // only allows travel in opposite direction to outside switch 
                        limTimeStep = nanos();
                    }

                    else if ((currentTimeStep - limTimeStep) < 10000000) // 10ms debounce time after switch is released, only allows travel in opposite direction to switch 
                    {
                        if (motorDir < 0) { motorDrive(); } // only allows travel in opposite direction to outside switch 
                    }

                    else { driveState = 1; }
                    
                    //printf("OUTERSWITCH INHIBITED.\n");

                    break;


                case 2: // if limitinside is pressed

                    if (!digitalRead(limitInside)) // if limitInside remains pressed
                    {
                        if (motorDir > 0) {motorDrive();} // only allows travel in opposite direction to inside switch 
                        limTimeStep = nanos();
                    }

                    else if ((currentTimeStep - limTimeStep) < 10000000) // 10ms debounce time after switch is released
                    {
                        if (motorDir > 0) {motorDrive();} // only allows travel in opposite direction to inside switch 
                    }

                    else { driveState = 1; }

                    break;
                    
                    //printf("INNERSWITCH INHIBITED.\n");


                case 0:

                    if ((currentTimeStep - prevTimeStep) > 10000000) // 10ms debounce time
                    {
                        driveState = ((2 * digitalRead(limitOutside)) - digitalRead(limitInside)); // re-evaluate drivestate during debounce if both limit switches are pressed
                    }

                    else{} // stop -- this would be if both limit switches were held down

                    break;

            }
        }


        void goalSpeed(float goalSpeed) // accelerate between current speed and desired speed at set acceleration rate
        {
            
        }

        bool goalPosition(uint16_t goalPosition, float speed, bool find) // 
        {
            if(goalPosition < stepCount)
            {
                if(speed != currentSpeed)
                {
                    setSpeed(speed);
                }

                controlLoop();
                return 1;
            }

            else if(goalPosition > stepCount)
            {
                setSpeed(speed*-1);
                
                if(speed != currentSpeed)
                {
                    setSpeed(speed);
                }

                controlLoop();
                return 1;
            }

            else{return 0;}
        }
        
        void goalPosition(uint16_t goalPosition, float speed) // incorperate acceleration and limit switch checks!!
        {

            if(goalPosition < stepCount)
            {
                setSpeed(speed);

                while(goalPosition < stepCount)
                {
                    controlLoop();
                }
            }

            else if(goalPosition > stepCount)
            {
                setSpeed(speed*-1);

                while(goalPosition > stepCount)
                {
                    controlLoop();
                }
            }

            else{}

        }
     
};


class EndEffectorConfig 
{
    private:

        const float stepPerRev = 3200.0;
        const float mmPerRev = 25.0;
        const float linkLength = 350.0;
        const float carriageOffset = (93.5 + 30.0 + 56.5); // inside edge of aluminum bloc to center of macron rail + aluminum block thickness + carriage pin to inside edge of carriage
        const float endOffset = 130.0; // center of end effector to outer linkage pin center
        const float linearStage = 250.0; // mm of travel possible for linear actuators

        float max(float x, float y)
        {
            x = abs(x);
            y = abs(y);
            
            if(x > y) {return x;}

            else {return y;}
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
        
            rightMotorSpeed = 0;
            leftMotorSpeed = 0;

            baseLength = 0; // length between outer pins on carriages
            basePartial = 0; // for right triangle of hypotenuse link length

            xPosition = 0;
            zPosition = 0;

            xDirection = 0;
            zDirection = 0;

        }



        // Position FUNCTIONS using IK and FK

        void updateMotorPosition() // converts left and right step counts to mm position (left = negative ; right = positive)
        {
            leftMotorPosition = ( ((float)(leftMotor.stepCount)) / stepPerRev) * mmPerRev;
            rightMotorPosition = ( ((float)(rightMotor.stepCount)) / stepPerRev) * mmPerRev;
        }

        void updateCurrentPosition() // get end effector XZ position from current motor steps (opposite of inverseKinematics)
        {

            updateMotorPosition();

            baseLength = (leftMotorPosition) + (carriageOffset * 2) + rightMotorPosition;
            basePartial = (baseLength - endOffset) / 2.0;

            zPosition = sqrt((linkLength * linkLength) - (basePartial * basePartial)) + 20;
            xPosition = ((leftMotorPosition*-1) + rightMotorPosition) / 2;

        }

        void goToPosition(float xCord, float zCord, float speed)
        {
            updateMotorPosition();

            zCord = zCord - 20.0; // z direction offset -- Frame of reference changed from the motor plane to the outer pin z cordinate with this operation

            baseLength = (2 * sqrt((linkLength * linkLength) - (zCord * zCord))) + (endOffset);

            float goalRightMotorPosition = (((2 * xCord) + baseLength) / 2.0) - carriageOffset; // goal right position in mm ( to be used for next line )
            float goalLeftMotorPosition = (baseLength - goalRightMotorPosition - (2*carriageOffset)); // goal left position in mm
            
            if ((zCord < 50) || zCord > 330.6 || goalRightMotorPosition > 250.0 || goalLeftMotorPosition > 250.0)
            {
                printf("Error Goal Position out of Bounds\n");
            }

            else
            {
                
                float leftDelta = abs(leftMotorPosition - goalLeftMotorPosition);
                float rightDelta = abs(rightMotorPosition - goalRightMotorPosition);
                
                leftMotorSpeed = speed * ((leftMotorPosition - goalLeftMotorPosition) / leftDelta);
                rightMotorSpeed = speed * ((rightMotorPosition - goalRightMotorPosition) / rightDelta);
                
                printf("\n\nLEFT speed: %f %\n", leftMotorSpeed);
                printf("\nRIGHT speed: %f %\n", rightMotorSpeed);
                
                if(leftDelta - rightDelta > 0)
                {
                    if(((rightDelta / leftDelta) * speed) >= 1) // avoid motor deadband
                    {
                        rightMotorSpeed = (rightDelta / leftDelta) * rightMotorSpeed; // partial speed to get there at same time as left
                    }
                }
                
                else if(leftDelta - rightDelta < 0)
                {
                    if(((leftDelta / rightDelta) * speed) >= 1) // avoid motor deadband
                    {
                        leftMotorSpeed = (leftDelta / rightDelta) * leftMotorSpeed; // partial speed to get there at same time as right
                    }
                }
                
                
                goalLeftMotorPosition = goalLeftMotorPosition * (stepPerRev / mmPerRev);
                goalRightMotorPosition = goalRightMotorPosition * (stepPerRev / mmPerRev);
                
                printf("\n\nLEFT speed: %f %\n", leftMotorSpeed);
                printf("\nRIGHT speed: %f %\n", rightMotorSpeed);
                
                bool findingLeft = 1;
                bool findingRight = 1;
                
                while(findingLeft || findingRight) 
                {
                    findingLeft = leftMotor.goalPosition(goalLeftMotorPosition, leftMotorSpeed, 1);
                    findingRight = rightMotor.goalPosition(goalRightMotorPosition, rightMotorSpeed, 1);
                }
            }

        }


        // Velocity Functions using method of kinematic coefficients + IK


        void directionalDrive(float CMD_X_DIR, float CMD_Z_DIR, float speed) // drives at set direction and speed (needs to be actively updating every step)
        {
            updateMotorPosition();

            baseLength = (leftMotorPosition) + (carriageOffset * 2) + rightMotorPosition;
            basePartial = (baseLength - endOffset) / 2.0;
            
            rightMotorSpeed = (CMD_X_DIR) - (CMD_Z_DIR * (sqrt((linkLength * linkLength) - (basePartial * basePartial)) / basePartial)); // find right speed from xz vectors + current posture
            leftMotorSpeed = ((2 * CMD_X_DIR) - rightMotorSpeed); // raw value (not in %)
            
            tempMax = max(rightMotorSpeed, leftMotorSpeed);
            rightMotorSpeed = (rightMotorSpeed / tempMax) * speed * -1; // converted to % speed commands (switch signs)
            leftMotorSpeed = (leftMotorSpeed / tempMax) * speed; // negative to keep frame of reference consistent with right motor (keep sign)

            leftMotor.setSpeed(leftMotorSpeed);
            rightMotor.setSpeed(rightMotorSpeed);

            leftMotor.controlLoop(); // activates motor control loop > drives with setSpeedCommands while watching limits
            rightMotor.controlLoop(); // activates motor control loop > drives with setSpeedCommands while watching limits

        }


        void calibrateZero(float speed)
        {
            float curSpeed = 1; // minimum Motor Speed
            uint16_t ACC_COUNT = 0; // start counting phase changes

            while (digitalRead(rightMotor.limitOutside))
            {
                if( !(ACC_COUNT % 100) && (curSpeed <= speed) )
                {
                    rightMotor.setSpeed(curSpeed*-1);
                    leftMotor.setSpeed(curSpeed);
                    curSpeed += 1;
                }

                leftMotor.motorDrive();
                rightMotor.motorDrive();

                ACC_COUNT += 1;
            }

            rightMotor.setStepPosition(32000-256);

            curSpeed = 1; // minimum Motor Speed
            ACC_COUNT = 0; // start counting phase changes

            while (digitalRead(leftMotor.limitOutside))
            {
                if( !(ACC_COUNT % 50) && (curSpeed <= speed) )
                {
                    leftMotor.setSpeed(curSpeed*-1);
                    rightMotor.setSpeed(curSpeed);
                    curSpeed += 1;
                }

                leftMotor.motorDrive();
                rightMotor.motorDrive();

                ACC_COUNT += 1;
            }

            leftMotor.setStepPosition(32000-256);
            
            updateCurrentPosition();

            goToPosition(0, 100, 30);
        }
        

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

};

uint64_t getNanos()
{
    struct  timespec ts;

    clock_gettime (CLOCK_MONOTONIC_RAW, &ts);

    return (uint64_t)ts.tv_sec * (uint64_t)1000000000 + (uint64_t)(ts.tv_nsec);
    
}



// main function
int main()
{   

    EndEffectorConfig mechanism(16000, 16000);

    //mechanism.moveInZ(50);

    mechanism.calibrateZero(25);
    
    delay(1000);
    
    //printf("\n\nL POSITION: %f mm\n", mechanism.leftMotorPosition);
    //printf("\nR POSITION: %f mm\n", mechanism.rightMotorPosition);
    mechanism.updateCurrentPosition();
    printf("\nX POSITION: %f mm\n", mechanism.xPosition);
    printf("\nZ POSITION: %f mm\n", mechanism.zPosition);
    
    uint64_t time1 = getNanos();
    
    while(getNanos() - time1 < 4000000000)
    {

        //mechanism.directionalDrive(100,100,10);

    }
    
    mechanism.updateCurrentPosition();
    
    printf("\nX POSITION: %f mm\n", mechanism.xPosition);
    printf("\nZ POSITION: %f mm\n", mechanism.zPosition);
   
}
