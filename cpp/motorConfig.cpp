#include <stdio.h>      // Used for printf() statements
#include <stdlib.h>
#include <wiringPi.h>   // Include WiringPi library!
#include <time.h>       // For NANOS function
#include <iostream>

using namespace std;

// to be created for left and right motor inside the endEffector Object
// commands for different motors instances can be used in the same while loop because we use relative time rather than delays to decide pulse timing (no need to multithread)

class MotorConfig
{   

    // private data members specific to our configuration -- dont want to ever modify
    private: 

        // mechanics of linear actuation//

        const float mmPerRev_XZ = 25.0; // milimeters linear travel per revolution of xz stepper motors
        const float mmPerRev_Y = 12.5; // milimeters linear travel per revolution of y stage stepper motor

        const uint16_t microStepsPerRev = 1600; // set on the steper drivers
        const uint32_t maxDelay = 303000; // nanoseconds -- max step delay before 0 speed command (MIN SPEED)
        const uint32_t minDelay = 3000; // nanoseconds -- min step delay representing max speed (MAX SPEED)

        // internal functions

        uint64_t nanos()
        {
            struct  timespec ts;

            clock_gettime (CLOCK_MONOTONIC_RAW, &ts);

            return (uint64_t)ts.tv_sec * (uint64_t)1000000000 + (uint64_t)(ts.tv_nsec);
        }


        bool checkLimitSwitch()
        {
            return (digitalRead(limitInside) && digitalRead(limitOutside));
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

        //float motorSpeed; // -100 to 100 set by "setspeed" 
        bool phase; // default motor state to low, this will toggle back and forth during driving -- might be able to delete this and just have off on pulse

        uint32_t currentDelay; // nanoseconds -- this indicates the speed of stepper motor and is updated with setSpeed funciton  (must be between Max and MinDelay)
        uint16_t currentPosition; // microsteps -- configured automatically during calibration by using limit switch as physical reference, updated every time step is pulsed 
        int8_t motorDir; // 1 for inward velocity  ||  -1 for outward velocity
        int8_t driveState; // 0 = limited | 1 = drive | 2 = inner drive | 3 = outer drive



        //** Goal (or desired) characterisitics for goal functions **//

        uint16_t goalPosition; // for goToPosition function -- this can be used later on for inverse kinematics and return to home feature
        float goalSpeed; // -100 to 100 set by "goalSpeed()" 


        // INSTANCE MODIFIER // 

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

            setSpeed(0); // ensuring setSpeed is at 0 so motors dont run when control loop starts
        }



        // SET FUNCTIONS //

        void setPulseDelay(uint32_t delay) // directly change current pulse delay in ns
        {
            currentDelay = delay; // ns
        }


        // This function should be run as a subscriber to motor % commands from the jetson NANO

        void setSpeed(float speed) // use value from -100 to 100 to represent % speed -- deadband -10% to +10% -- see motorConfig.pdf for speed charts
        {
            if(speed >= 10 && speed <= 100)
            {
                digitalWrite(dirPin, 1); // if speed is positive, then drive inward
                currentDelay = uint32_t(4000000.0 / (1.6*speed)); // translate speed command to step delay in ns
                motorDir = 1; // inwards
                //motorSpeed = speed; // allows other classes / subscribers to view current setSpeed
            }

            else if(speed <= -10 && speed >= -100)
            {
                digitalWrite(dirPin, 0); // if speed is negative, then drive outward
                currentDelay = uint32_t(4000000.0 / (-1.6*speed)); // translate speed command to step delay in ns
                motorDir = -1; // outwards
                //motorSpeed = speed;
            }

            else
            {
                currentDelay = 0;
                //motorSpeed = 0.0;
            }

        }


        void setGoalSpeed(float speed)
        {

            // utilize acceleration between commanded speeds
        }


        void setAcceleration(float acceleration)
        {
            // set global var
        }



        // MOTOR CONTROL FUNCTIONS //

        void moveIn()
        {
            digitalWrite(dirPin, 1);

            if(nanos() - prevTimeStep > currentDelay)
            {
                digitalWrite(stepPin, phase);
                prevTimeStep = nanos();
                currentPosition += (1*phase);
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
                currentPosition -= (1*phase);
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
                currentPosition += (motorDir*phase);
                phase = !phase;
            }

        }

        // TO BE USED IN ROS Motor Control NODE -- updating at 0.5-1GHz
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
                        if (motorDir > 0) {motorDrive();} // only allows travel in opposite direction to outside switch 
                        limTimeStep = nanos();
                    }

                    else if ((currentTimeStep - limTimeStep) < 10000000) // 10ms debounce time after switch is released, only allows travel in opposite direction to switch 
                    {
                        if (motorDir > 0) { motorDrive(); } // only allows travel in opposite direction to outside switch 
                    }

                    else { driveState = 1; }
                    
                    //printf("OUTERSWITCH INHIBITED.\n");

                    break;


                case 2: // if limitinside is pressed

                    if (!digitalRead(limitInside)) // if limitInside remains pressed
                    {
                        if (motorDir < 0) {motorDrive();} // only allows travel in opposite direction to inside switch 
                        limTimeStep = nanos();
                    }

                    else if ((currentTimeStep - limTimeStep) < 10000000) // 10ms debounce time after switch is released
                    {
                        if (motorDir < 0) {motorDrive();} // only allows travel in opposite direction to inside switch 
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


        void goalDrive() // based on motor drive, but inlcude auto ACCELRATION between commanded speeds
        {
            
        }

        void goToPosition(uint16_t goalPosition, float speed) // incorperate acceleration!!
        {

            if(goalPosition < currentPosition)
            {
                setSpeed(speed*-1);

                while(goalPosition < currentPosition)
                {
                    motorDrive();
                }
            }

            else if(goalPosition > currentPosition)
            {
                setSpeed(speed);

                while(goalPosition > currentPosition)
                {
                    motorDrive();
                }
            }

            else{}

        }
     
};

uint64_t nanos1()
{
    struct  timespec ts;

    clock_gettime (CLOCK_MONOTONIC_RAW, &ts);

    return (uint64_t)ts.tv_sec * (uint64_t)1000000000 + (uint64_t)(ts.tv_nsec);
}



// main function
int main()
{   
    MotorConfig leftMotor(17, 18, 5, 1);
    MotorConfig rightMotor(4, 27, 2, 3); 

    float speedLeft = -15.0;
    float speedRight = -15.0;

    leftMotor.setSpeed(speedLeft);
    rightMotor.setSpeed(speedRight);
    
    printf("\nLeft DELAY: %d ns\n", leftMotor.currentDelay);
    printf("\nRIGHT DELAY: %d ns\n", rightMotor.currentDelay);

    uint64_t time1 = nanos1();
    
    while(nanos1() - time1 < 1000000000)
    {

        rightMotor.controlLoop();
        leftMotor.controlLoop();

    }
    
    printf("\nLeft DELAY: %d ns\n", leftMotor.currentDelay);
    printf("\nRIGHT DELAY: %d ns\n", rightMotor.currentDelay);
   
}













