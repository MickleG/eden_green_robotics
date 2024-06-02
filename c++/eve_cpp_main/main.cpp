#include <stdio.h> 
#include <stdlib.h>
#include <time.h>       // For NANOS function
#include <stdint.h>
#include <cmath>        // For sqrt and other math functions
#include <time.h>
#include <wiringPi.h>

#include <EndEffectorConfig.h>
#include <MotorConfig.h>
//#include <XM430.h>
//#include <Cutter.h>

using namespace std;

uint64_t getNanos()
{
    struct  timespec ts;

    clock_gettime (CLOCK_MONOTONIC_RAW, &ts);

    return (uint64_t)ts.tv_sec * (uint64_t)1000000000 + (uint64_t)(ts.tv_nsec);
    
}

// main function
int main()
{   

    // INITIALIZE ALL MOTORS AND OBJECTS//

    EndEffectorConfig mechanism(0, 0); // defines the positioning mechanism

    // UPDATE Y STAGE PINS
    //MotorConfig yStage(-1, -1, -1, -1); // defines the y stage motor, will need to determine if up/down is positive or negative speed


    // CALIBRATE XZ // *determine if successful
    mechanism.calibrateZero(40);
    
    delay(500);
    
    mechanism.updateCurrentPosition();
    printf("\nX POSITION: %f mm\n", mechanism.xPosition);
    printf("\nZ POSITION: %f mm\n", mechanism.zPosition);
   
}