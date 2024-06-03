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

        const float deadBandSpeed = 1.0; 
        const uint64_t debounceTime = 1000000; // 1 ms

        uint64_t accTime; // nanoseconds -- logged when acceleration added to speed
        uint64_t accInterval = 10000000; // nanoseconds -- interval to increase currentSpeed

        // get time in nanoseconds based on monatonic clock (more accurate)
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
        
        float currentSpeed; // -160 to 160 set by setSpeed()
        float acceleration; // m/s per accInterval, in 
        float goalSpeed; // 0 to 160 set by setSpeedMagnitude()

        uint16_t goalPosition;
        int32_t goalSteps;
        uint16_t remainingSteps;
        uint16_t accSteps;
 
        bool findingTarget; // boolean for if motor is actively seaking a goal position
        
        float defaultAcceleration = 0.05; // m/s per step

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
            accTime = nanos();

            phase = 0;
            driveState = 1;
            stepCount = 0; 
            acceleration = 0.01;
            accSteps = 0;

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
            accTime = nanos();
            
            phase = 0; // initialize stepper motor phase
            driveState = 1; // initialize driveState to normal (should be initialized to 1 after calibration) 
            stepCount = 0; // intialize motor position
            acceleration = 0.01;
            accSteps = 0;

            setSpeed(0);
        }

        MotorConfig()
        {
            prevTimeStep = nanos(); // sets initial pulse time instance
            limTimeStep = nanos(); // sets initial limit trigger time instance 
            currentTimeStep = nanos();
            accTime = nanos();

            phase = 0; // initialize stepper motor phase
            driveState = 1; // initialize driveState to normal (should be initialized to 1 after calibration) 
            stepCount = 0; 
            acceleration = 0.01;
            accSteps = 0;

            setSpeed(0); // ensuring setSpeed is at 0 so motors dont run when control loop starts
        }


        ///////////////////////////////////////////////////////////////////////
        // SET FUNCTIONS //
        ///////////////////////////////////////////////////////////////////////

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


        // setSpeed() sets the step delay (dictates motor speed), motor direction, and logs the prev in global vars

        void setSpeed(float speed) // use value from -100 to 100 to represent % speed -- deadband -1% to +1% -- see motorConfig.pdf for speed charts
        {
            
            if((speed >= deadBandSpeed) && speed <= 160)
            {
                digitalWrite(dirPin, 1); // if speed is positive, then drive inward
                currentDelay = (uint32_t)(4000000.0 / speed); // translate speed command to step delay in ns
                motorDir = -1; // inwards
                currentSpeed = speed;
            }

            else if((speed <= (-1 * deadBandSpeed)) && speed >= -160)
            {
                digitalWrite(dirPin, 0); // if speed is negative, then drive outward
                currentDelay = (uint32_t)(-4000000.0 / speed); // translate speed command to step delay in ns
                motorDir = 1; // outwards
                currentSpeed = speed;
            }

            else
            {
                currentDelay = 0;
                currentSpeed = 0;
            }

        }

        void setSpeedMagnitude(float speed)
        {
            if(speed >= deadBandSpeed && speed <= 160)
            {
                currentDelay = (uint32_t)(4000000.0 / speed); // translate speed command to step delay in ns
                currentSpeed = speed;
            }  

            else 
            {
                currentDelay = 0;
                currentSpeed = 0;
            }

        }


        void setAcceleration(float acc)
        {
            acc = acc * (((float)accInterval) / 1000000000.0);

            acceleration = acc; 
        }


        void setDefaultAcceleration(float acc)
        {
            defaultAcceleration = acc; 
        }


        void setGoalPosition(uint16_t position, float speed)
        {
            setSpeedMagnitude(deadBandSpeed);

            goalPosition = position;
            goalSteps = abs(position - stepCount);
            goalSpeed = speed;
            accSteps = 0;

            if(goalSteps > 0)
            {
                findingTarget = 1;
                motorDir = (goalSteps / (position - stepCount));

                if(motorDir > 0) {digitalWrite(dirPin, 0);}
                else {digitalWrite(dirPin, 1);}
                
            }

        }


        ///////////////////////////////////////////////////////////////////////
        // MOTOR CONTROL FUNCTIONS
        ///////////////////////////////////////////////////////////////////////

        void moveIn(uint64_t delay) // no limit sw monitoring
        {
            digitalWrite(dirPin, 1);

            if(nanos() - prevTimeStep > delay)
            {
                digitalWrite(stepPin, phase);
                prevTimeStep = nanos();
                stepCount += (1*phase);
                phase = !phase;
            }
        }



        void moveOut(uint64_t delay) // no limit sw monitoring
        {
            digitalWrite(dirPin, 0);

            if((nanos() - prevTimeStep > delay) && digitalRead(limitOutside) && digitalRead(limitInside))
            {
                digitalWrite(stepPin, phase);
                prevTimeStep = nanos();
                stepCount -= (1*phase);
                phase = !phase;
            }
        }



        // TO BE USED IN CONTROL LOOP -- decides when its time to step the motor based on setSpeed, keeps track of position as well
        void motorDrive()
        {
            currentTimeStep =  nanos(); // SEE IF THIS WORKS BETTER

            if( ((currentTimeStep - prevTimeStep) >= currentDelay) && (currentDelay > 0) ) // changed > to >= for currentDelay
            {
                digitalWrite(stepPin, phase); // motor coils HIGH or LOW (1 or 0)
                prevTimeStep = nanos(); // log the time stamp for step
                stepCount += (motorDir*phase); // only add stepCount when coils on (phase = 1)
                phase = !phase; // switch phase b/t 0 and 1 
            }

        }


        // TO BE USED IN ROS Motor Control NODE -- update at 1GHz->2GHz
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
                        else {currentSpeed = 0;} // report 0 motor speed even though speed commanded might be positive

                        limTimeStep = nanos();
                    }

                    else if ((currentTimeStep - limTimeStep) < debounceTime) // 10ms debounce time after switch is released, only allows travel in opposite direction to switch 
                    {
                        if (motorDir < 0) { motorDrive(); } // only allows travel in opposite direction to outside switch 
                        else {currentSpeed = 0;}
                    }

                    else { driveState = 1; }
                    
                    //printf("OUTERSWITCH INHIBITED.\n");

                    break;


                case 2: // if limitinside is pressed

                    if (!digitalRead(limitInside)) // if limitInside remains pressed
                    {
                        if (motorDir > 0) {motorDrive();} // only allows travel in opposite direction to inside switch 
                        else {currentSpeed = 0;} // report 0 motor speed even though speed commanded might be negative

                        limTimeStep = nanos();
                    }

                    else if ((currentTimeStep - limTimeStep) < debounceTime) // 10ms debounce time after switch is released
                    {
                        if (motorDir > 0) {motorDrive();} // only allows travel in opposite direction to inside switch 
                        else {currentSpeed = 0;}
                    }

                    else { driveState = 1; }

                    break;
                    
                    //printf("INNERSWITCH INHIBITED.\n");


                case 0:

                    if ((currentTimeStep - prevTimeStep) > debounceTime) // 10ms debounce time
                    {
                        driveState = ((2 * digitalRead(limitOutside)) - digitalRead(limitInside)); // re-evaluate drivestate during debounce if both limit switches are pressed
                    }

                    else{currentSpeed = 0;} // stop -- this would be if both limit switches were held down

                    break;

            }
        }



        ///////////////////////////////////////////////////////////////////////
        // goal and acceleration functions
        ///////////////////////////////////////////////////////////////////////

        void accToSpeed(float speed) // ACCELERATION SETSPEED() ALTERNATIVE -- use in ROS loop which subscribes to commanded speed, true derivative of velocity
        {

            if((currentSpeed < speed) && ((nanos() - accTime) >= accInterval))
            {
                setSpeed(currentSpeed + acceleration);
                accTime = nanos();
            }

            else if((currentSpeed > speed) && ((nanos() - accTime) >= accInterval))
            {
                setSpeed(currentSpeed - acceleration);
                accTime = nanos();
            }

        }


        void goalPositionOld(uint16_t goalPos, float speed) // for multiple motors at a time -- use return value to see when position is achieved
        {
            if(goalPos < stepCount)
            {
                if(speed != currentSpeed)
                {
                    setSpeed(speed);
                }

                controlLoop();
            }

            else if(goalPos > stepCount)
            {
                
                if( (speed*-1) != currentSpeed )
                {
                    setSpeed(speed *-1);
                }

                controlLoop();
            }

            else{findingTarget = 0;}
        }


        void goalPositionACC() // for multiple motors at a time -- use return value to see when position is achieved
        {
            
            if(findingTarget)
            {

                remainingSteps = abs(goalPosition - stepCount);
                if((remainingSteps > (goalSteps / 2)) && (currentSpeed < goalSpeed))
                {
                    controlLoop();
                    
                    if((currentTimeStep - prevTimeStep) >= currentDelay)
                    {
                        setSpeedMagnitude(currentSpeed + (phase*defaultAcceleration));
                        accSteps += phase;
                        //printf("\n acc rem steps : %d \n", remainingSteps);
                        //printf("\n acc steps : %d \n", accSteps);
                        
                    }
                    
                }
                
                else if (remainingSteps >= accSteps) 
                {
                    controlLoop();
                    
                    if ((currentTimeStep - prevTimeStep) >= currentDelay)
                    {
                        //printf("\n  cur Speed : %f \n", currentSpeed);
                        //printf("\n const rem steps : %d \n", remainingSteps);
                    }
                }

                else if ( (remainingSteps < accSteps) && (remainingSteps > 0) && (currentSpeed > (deadBandSpeed + defaultAcceleration)) )
                {
                    controlLoop();
                    
                    if ((currentTimeStep - prevTimeStep) >= currentDelay)
                    {
                        setSpeedMagnitude(currentSpeed - (phase*defaultAcceleration));
                        //printf("\n  cur Speed : %f \n", currentSpeed);
                        //printf("\n dec rem steps : %d \n", remainingSteps);
                        //printf("\n acc steps : %d \n", accSteps);
                    }
                }

                else
                {
                    accSteps = 0; // reset accSteps for next goal position call
                    findingTarget = 0; // position is found
                }
            }
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

            float goalRightMotorPosition = (((2 * xCord) + baseLength) * 0.5) - carriageOffset; // goal right position in mm ( to be used for next line )
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
                
                leftMotor.setGoalPosition(goalLeftMotorPosition, abs(leftMotorSpeed));
                rightMotor.setGoalPosition(goalRightMotorPosition, abs(rightMotorSpeed));

                while(rightMotor.findingTarget || leftMotor.findingTarget) 
                {
                    leftMotor.goalPositionACC(); // TEST WITH ACC
                    rightMotor.goalPositionACC(); // TEST WITH ACC
                }
            }

        }

        void calibrateZero(float speed) // NEEDS UPDATED FOR ACCELERATION
        {
            rightMotor.setSpeed(0);
            leftMotor.setSpeed(0);

            while (digitalRead(rightMotor.limitOutside)) // check right switch
            {

                rightMotor.setSpeed(speed*-1); // move right relative to robot
                leftMotor.setSpeed(speed); // move right relative to robot

                leftMotor.motorDrive(); // probably safer to use this even tho not needed
                rightMotor.motorDrive();
            }

            rightMotor.setStepPosition(32000-256); // because the switch usually hits about 2 mm from actually touching end


            while (digitalRead(leftMotor.limitOutside))
            {
                leftMotor.setSpeed(speed*-1);
                rightMotor.setSpeed(speed);

                leftMotor.motorDrive();
                rightMotor.motorDrive();
            }

            leftMotor.setStepPosition(32000-256);
            
            updateCurrentPosition();

            goToPosition(0, 100, 50);
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


        void directionalDrive(float CMD_X_DIR, float CMD_Z_DIR, float speed) // drives at set direction and speed (needs to be actively updating every step)
        {
            updateMotorPosition();

            baseLength = (leftMotorPosition) + (carriageOffset * 2) + rightMotorPosition; // distance between left carriage and right carriage
            basePartial = (baseLength - endOffset) * 0.5; // right triangle component along linear actuator axis (linkage length is hyptenuse, z cord is height)
            
            rightMotorSpeed = (CMD_X_DIR) - (CMD_Z_DIR * (sqrt((linkLength * linkLength) - (basePartial * basePartial)) / basePartial)); // where CMDXDIR = dx/dt and CMDZDIR = dz/dt > finding dRM/dt (this value * 1.6 = mm/s)
            leftMotorSpeed = ((2 * CMD_X_DIR) - rightMotorSpeed); // using dRM/dt to get dLM/dt (this value * 1.6 = mm/s)
            
            tempMax = maxAbs(rightMotorSpeed, leftMotorSpeed); // find the motor with greater speed magnitude
            rightMotorSpeed = (rightMotorSpeed / tempMax) * speed * -1; // convert motor speed to speed commanded by function input argument "speed" (switch signs)
            leftMotorSpeed = (leftMotorSpeed / tempMax) * speed; // negative to keep frame of reference consistent with right motor (keep sign)

            leftMotor.setSpeed(leftMotorSpeed); // declaring left motor speed
            rightMotor.setSpeed(rightMotorSpeed); // declaring right motor speed
            
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

    EndEffectorConfig mechanism(20000, 20000);

    mechanism.calibrateZero(40);
    //mechanism.rightMotor.setGoalPosition(16000, 30);
    
    while(mechanism.rightMotor.findingTarget) 
    {
        mechanism.rightMotor.goalPositionACC(); // TEST WITH ACC
    }
    
    delay(1000);
    
    mechanism.updateCurrentPosition();
    printf("\nX POSITION: %f mm\n", mechanism.xPosition);
    printf("\nZ POSITION: %f mm\n", mechanism.zPosition);
    
    uint64_t time1 = getNanos();
    
    while(getNanos() - time1 < 3000000000)
    {

        mechanism.directionalDrive(-100,100,40);
    }
    
    mechanism.updateCurrentPosition();
    
    printf("\nX POSITION: %f mm\n", mechanism.xPosition);
    printf("\nZ POSITION: %f mm\n", mechanism.zPosition);
   
}
