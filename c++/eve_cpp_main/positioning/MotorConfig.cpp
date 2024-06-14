// include header file for this Class
#include "MotorConfig.h"
#include <stdint.h>     // For uint8_t, uint16_t, uint32_t, uint64_t
#include <stdlib.h>
#include <stdio.h>      // For printf statements
#include <wiringPi.h>   // Include WiringPi library
#include <time.h>       // For NANOS function
#include <cmath>        // For sqrt and other math functions

using namespace std;

// to be created for left and right motor inside the endEffector Object
// commands for different motors instances can be used in the same while loop because we use relative time rather than delays to decide pulse timing (no need to multithread)

//class MotorConfig

    // private data members specific to our configuration -- dont want to ever modify

    // get time in nanoseconds based on monatonic clock (more accurate)
    uint64_t MotorConfig::nanos()
    {
        struct  timespec ts;

        clock_gettime (CLOCK_MONOTONIC_RAW, &ts);

        return (uint64_t)ts.tv_sec * (uint64_t)1000000000 + (uint64_t)(ts.tv_nsec);
    }


    // public member function    

    // INSTANCE MODIFIERS // 

    MotorConfig::MotorConfig(char Side)
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
        switchPress = 0;
        driveState = 1;
        stepCount = 0; 
        accSteps = 0;
        acceleration = 0.1;

        setSpeed(0);

    }

    MotorConfig::MotorConfig(uint8_t s, uint8_t d, uint8_t limOut, uint8_t limIn)
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
        switchPress = 0;
        driveState = 1; // initialize driveState to normal (should be initialized to 1 after calibration) 
        stepCount = 0; // intialize motor position
        accSteps = 0;
        acceleration = 0.1;

        setSpeed(0);
    }

    MotorConfig::MotorConfig()
    {
        prevTimeStep = nanos(); // sets initial pulse time instance
        limTimeStep = nanos(); // sets initial limit trigger time instance 
        currentTimeStep = nanos(); // intialize current time step
        accTime = nanos(); // intialize acceleration time step

        phase = 0; // initialize stepper motor phase
        switchPress = 0; // assume no switches are pressed during startup
        driveState = 1; // initialize driveState to normal (should be initialized to 1 after calibration) 
        stepCount = 0; // initialize motor position to 0 (waiting on calibration)
        accSteps = 0;
        acceleration = 0.1;

        setSpeed(0); // ensuring setSpeed is at 0 so motors dont run when control loop starts
    }


    ///////////////////////////////////////////////////////////////////////
    // SET FUNCTIONS //
    ///////////////////////////////////////////////////////////////////////

    void MotorConfig::setHardware(uint8_t s, uint8_t d, uint8_t limOut, uint8_t limIn)
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


    
    void MotorConfig::setStepPosition(uint16_t steps)
    {
        stepCount = steps;
    }


    void MotorConfig::setPulseDelay(uint32_t delay) // directly change current pulse delay in ns
    {
        currentDelay = delay; // ns
    }


    // setSpeed() sets the step delay (dictates motor speed), motor direction, and logs the prev in global vars
    void MotorConfig::setSpeed(float speed) // use value from -100 to 100 to represent % speed -- deadband -1% to +1% -- see motorConfig.pdf for speed charts
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

    void MotorConfig::setSpeedMagnitude(float speed)
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


    void MotorConfig::setAcceleration(float acc)
    {
        acc = acc * (((float)accInterval) / 1000000000.0);

        acceleration = acc; 
    }


    void MotorConfig::setGoalPosition(uint16_t position, float speed)
    {
        setSpeedMagnitude(deadBandSpeed);

        goalStepPosition = position;
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

    void MotorConfig::moveIn(uint64_t delay) // no limit sw monitoring
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



    void MotorConfig::moveOut(uint64_t delay) // no limit sw monitoring
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
    void MotorConfig::motorDrive()
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

    void MotorConfig::motorDriveY()
    {
        currentTimeStep =  nanos(); // SEE IF THIS WORKS BETTER

        if( ((currentTimeStep - prevTimeStep) >= currentDelay) && (currentDelay > 0) ) // changed > to >= for currentDelay
        {
            digitalWrite(stepPin, phase); // motor coils HIGH or LOW (1 or 0)
            prevTimeStep = nanos(); // log the time stamp for step
            stepCount += (-motorDir*phase); // only add stepCount when coils on (phase = 1)
            phase = !phase; // switch phase b/t 0 and 1 
        }

    }


    // TO BE USED IN ROS Motor Control NODE -- update at 1GHz->2GHz
    void MotorConfig::controlLoop()
    {
        currentTimeStep = nanos(); // test if this works better, as this is also updated inside motorDrive
        
        bool outerSwitch = digitalRead(limitOutside);
        bool innerSwitch = digitalRead(limitInside);


        switch (driveState)
        {
            case 1:

                motorDrive(); // run the motor at setspeed until switch is determed to be pressed in the line below

                if(!innerSwitch || !outerSwitch)
                {
                    if(!switchPress)
                    {
                        limTimeStep = nanos();
                        switchPress = 1;
                    }

                    else if (currentTimeStep - limTimeStep > debounceTime)
                    {
                        driveState = ((2 * outerSwitch) - innerSwitch); // will determine which precise state we are in
                    }
                }

                else
                {
                    switchPress = 0; // toggle switch pressed boolean to 0 as there is no limit switch pressed
                }

                break;


            case -1: // if limitOutside is pressed

                if (!outerSwitch) // if limitOutside remains pressed
                {
                    if (motorDir < 0) { motorDrive(); } // only allows travel in opposite direction to outside switch 
                    else {currentSpeed = 0;} // report 0 motor speed even though speed commanded might be positive

                    if (!innerSwitch) { driveState = 0; }

                    limTimeStep = nanos();
                }

                else if ((currentTimeStep - limTimeStep) < debounceTime) // 1ms debounce time after switch is released, only allows travel in opposite direction to switch 
                {
                    if (motorDir < 0) { motorDrive(); } // only allows travel in opposite direction to outside switch 
                    else {currentSpeed = 0;}
                }

                else { driveState = 1; }
                
                break;


            case 2: // if limitinside is pressed

                if (!innerSwitch) // if limitInside remains pressed
                {
                    if (motorDir > 0) { motorDrive(); } // only allows travel in opposite direction to inside switch 
                    else { currentSpeed = 0; } // report 0 motor speed even though speed commanded might be negative

                    if(!outerSwitch) { driveState = 0; }

                    limTimeStep = nanos();
                }

                else if ((currentTimeStep - limTimeStep) < debounceTime) // 1ms debounce time after switch is released
                {
                    if (motorDir > 0) { motorDrive(); } // only allows travel in opposite direction to inside switch 
                    else { currentSpeed = 0; }
                }

                else { driveState = 1; }

                break;


            case 0:

                if ((currentTimeStep - prevTimeStep) > debounceTime) // 1ms debounce time
                {
                    driveState = ((2 * outerSwitch) - innerSwitch); // re-evaluate drivestate during debounce if both limit switches are pressed
                }

                else{currentSpeed = 0;} // stop -- this would be if both limit switches were held down

                break;

        }
    }



    ///////////////////////////////////////////////////////////////////////
    // goal and acceleration functions
    ///////////////////////////////////////////////////////////////////////

    void MotorConfig::accToSpeed(float speed) // ACCELERATION SETSPEED() ALTERNATIVE -- use in ROS loop which subscribes to commanded speed, true derivative of velocity
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


    void MotorConfig::goalPosition() // for multiple motors at a time -- use return value to see when position is achieved
    {
        
        if(findingTarget)
        {

            uint16_t remainingSteps = abs(goalStepPosition - stepCount);

            if((remainingSteps > (goalSteps / 2)) && (currentSpeed < goalSpeed))
            {
                controlLoop();
                
                if((currentTimeStep - prevTimeStep) >= currentDelay)
                {
                    setSpeedMagnitude(currentSpeed + (phase*goalAcceleration));
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

            else if ( (remainingSteps < accSteps) && (remainingSteps > 0) && (currentSpeed > (deadBandSpeed + goalAcceleration)) )
            {
                controlLoop();
                
                if ((currentTimeStep - prevTimeStep) >= currentDelay)
                {
                    setSpeedMagnitude(currentSpeed - (phase*goalAcceleration));
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