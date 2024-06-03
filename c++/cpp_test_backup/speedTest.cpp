#include <stdio.h>    // Used for printf() statements
#include <wiringPi.h> // Include WiringPi library!
#include <stdint.h>
#include <time.h>
#include <inttypes.h>

// Pin number declarations. We're using the Broadcom chip pin numbers.

const int rightStepPin = 4; // right motor step, relative to robot camera
const int rightDirPin = 27;  // right motor direction, High = moves inward towards RPI, 

const int leftStepPin = 17; // right motor step, relative to robot camera
const int leftDirPin = 18;  // right motor direction, High = moves inward towards RPI, 


const int rightOutLimit = 2; // Active-low button - Broadcom pin 17, P1 pin 11
const int rightInLimit = 3; // Active-low button - Broadcom pin 17, P1 pin 11
const int leftOutLimit = 5; // Active-low button - Broadcom pin 17, P1 pin 11
const int leftInLimit = 1; // Active-low button - Broadcom pin 17, P1 pin 11

// need to test motor


uint64_t nanos()
{
    struct  timespec ts;

    clock_gettime (CLOCK_MONOTONIC_RAW, &ts);

    return (uint64_t)ts.tv_sec * (uint64_t)1000000000 + (uint64_t)(ts.tv_nsec);
}



int main(void)
{
    // Setup stuff:
    wiringPiSetupGpio(); // Initialize wiringPi -- using Broadcom pin numbers

    pinMode(rightStepPin, OUTPUT); 	// Set right motor step pin to output mode
    pinMode(rightDirPin, OUTPUT);     	// Set right motor direction pin to output mode
    //pinMode(leftStepPin, OUTPUT); 	// Set left motor step pin to output mode
    //pinMode(leftDirPin, OUTPUT);     	// Set left motor direction pin to output mode
    
    pinMode(rightOutLimit, INPUT);      	
    pullUpDnControl(rightInLimit, PUD_UP); 
    
    pinMode(rightInLimit, INPUT);      	   
    pullUpDnControl(rightInLimit, PUD_UP); 
    
    //pinMode(leftInLimit, INPUT);      	
    //pullUpDnControl(leftInLimit, PUD_UP); 
    
    //pinMode(leftOutLimit, INPUT);      	 
    //pullUpDnControl(leftOutLimit, PUD_UP); 

    uint64_t previousTimeStep = nanos(); // nanoseconds -- logged immediately after a pulse has happened
    uint64_t currentTimeStep; // nanoseconds -- updated at beginning of every loop
    
    uint64_t testTimeInit; // nanoseconds -- logged immediately after a pulse has happened

    int currentPosition;
    bool phase = 0;
    
    

    uint32_t currentDelay = 115000; // *** INDEPENDED VARIBALE *** in nanoseconds
    
    digitalWrite(rightDirPin, 1); // TEST CRITICAL

    printf("SCRIPT IS RUNNING.\n");
    
    bool on = 1; 

    while(on)
    {
        if (digitalRead(leftOutLimit)) // TEST CRITICAL
        {
            currentTimeStep = nanos();
            if((currentTimeStep - previousTimeStep) > 1000000000)
            {
                printf("PRESS LIMIT SWITCH TO INTIATE COUNTDOWN.\n");
                previousTimeStep = nanos();
            }
        }
    	else if (digitalRead(rightOutLimit) == 0)  // TEST CRITICAL
    	{ 
            currentPosition = 0;

            printf("3.\n");
            delay(1000);
            printf("2.\n");
            delay(1000);
            printf("1.\n");
            delay(1000);
            printf("GOOOOOOOOO.\n");

            previousTimeStep = nanos();
            testTimeInit = nanos();

            while (digitalRead(rightInLimit) && digitalRead(leftOutLimit)) // TEST CRITICAL
            {
                currentTimeStep =  nanos();

                if((currentTimeStep - previousTimeStep) > currentDelay)
                {
                    digitalWrite(rightStepPin, phase); // TEST CRITICAL
                    previousTimeStep = nanos();
                    currentPosition += (1*phase);
                    phase = !phase;
                }
            }

            on = 0;
    	}
        else
        {
            currentTimeStep = nanos();
            if((currentTimeStep - previousTimeStep) > 1000000000)
            {
                printf("MOVE MOTOR TO TEST POSITION.\n");
                previousTimeStep = nanos();
            }
        }
    }
    
    double time1 = double(currentTimeStep - testTimeInit) / 1000000000.0;
    double speed1 = 250.0 / time1;
    
    printf("\n\n--------------------------------------\n");
    printf("\nDELAY: %d ns\n", currentDelay);
    printf("TIME: %f s\n", time1);
    printf("AVERAGE SPEED: %f mm/s\n", speed1);
    printf("STEPS: %d\n", currentPosition);
    printf("\n--------------------------------------\n");
    
    // RETURN TO HOME!!
    
    delay(1000);
    printf("\n\nRETURN\n");
    
    digitalWrite(rightDirPin, 0); // TEST CRITICAL
    testTimeInit = nanos();
    uint64_t timeDiff = 0;
    
    while (digitalRead(rightOutLimit) && digitalRead(leftOutLimit)) // TEST CRITICAL
    {
        currentTimeStep =  nanos();

        if((currentTimeStep - previousTimeStep) > currentDelay)
        {
            timeDiff = (currentTimeStep - previousTimeStep);
            digitalWrite(rightStepPin, phase);
            previousTimeStep = nanos();
            currentPosition -= (1*phase);
            phase = !phase;
        }
    }
    
    time1 = double(currentTimeStep - testTimeInit) / 1000000000.0;
    
    printf("\nRETURN TIME: %f s\n", time1);
    printf("\nSteps Skipped: %d\n\n", currentPosition);
    printf("TIME ERROR: " "%" PRIu64 " ns\n", timeDiff - (uint64_t(currentDelay)));

    return 0;
}
