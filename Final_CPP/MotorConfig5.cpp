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

        //** Time Step for accurate pulsing of stepper motors in nanoseconds **//

        const float deadBandSpeed = 1.0; 
        const uint64_t debounceTime = 1000000; // 1 ms
        const float defaultAcceleration = 0.05; // m/s per step

        uint64_t currentTimeStep; // nanoseconds -- logged at the beggining of motor control loop function
        uint64_t prevTimeStep; // nanoseconds -- logged immediately after a pulse has happened
        uint64_t limTimeStep; // nanoseconds -- logged immediately after limit switch is triggered, used for debounce time

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
        uint8_t accSteps;
 
        bool findingTarget; // boolean for if motor is actively seaking a goal position

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
            
            if(speed >= deadBandSpeed && speed <= 160)
            {
                digitalWrite(dirPin, 1); // if speed is positive, then drive inward
                currentDelay = (uint32_t)(4000000.0 / speed); // translate speed command to step delay in ns
                motorDir = -1; // inwards
                currentSpeed = speed;
            }

            else if(speed <= (-1 * deadBandSpeed) && speed >= -160)
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


        void setGoalSteps(uint16_t position, float speed)
        {
            setSpeed(deadBandSpeed);

            goalPosition = position;
            goalSteps = abs(position - stepCount);
            goalSpeed = speed;

            if(goalSteps > 0)
            {
                findingTarget = 1;
                motorDir = (goalSteps / (position - stepCount)) * -1;

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
            //currentTimeStep =  nanos(); // SEE IF THIS WORKS BETTER

            if( ((currentTimeStep - prevTimeStep) >= currentDelay) && (currentSpeed > 0) ) // changed > to >= for currentDelay
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

            remainingSteps = abs(goalPosition - stepCount);

            if((remainingSteps < (goalSteps / 2)) && (currentSpeed < goalSpeed))
            {

                setSpeedMagnitude(currentSpeed + defaultAcceleration);
                controlLoop();

                accSteps +=1;
            }

            else if ( (remainingSteps <= accSteps) && (currentSpeed >= (deadBandSpeed + defaultAcceleration)) )
            {
                setSpeedMagnitude(currentSpeed - defaultAcceleration);
                controlLoop();
            }

            else if (remainingSteps > 0) {controlLoop();}

            else
            {
                accSteps = 0; // reset accSteps for next goal position call
                findingTarget = 0; // position is found
            }
        }
     
};