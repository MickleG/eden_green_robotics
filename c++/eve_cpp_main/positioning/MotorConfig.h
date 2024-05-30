#ifndef MOTORCONFIG_H
#define MOTORCONFIG_H

#include <stdint.h>  // For uint8_t, uint16_t, uint32_t, uint64_t
#include <wiringPi.h>   // Include WiringPi library
#include <time.h>       // For NANOS function
#include <cmath>        // For sqrt and other math functions
#include <iostream>     // For standard I/O streams

class MotorConfig
{
private:

    const float deadBandSpeed = 1.0;
    const uint64_t debounceTime = 1000000; // 1 ms
    const float goalAcceleration = 0.1; // m/s per step

    uint64_t accInterval = 1000000; // nanoseconds -- interval to increase currentSpeed

    // Time Step for accurate pulsing of stepper motors in nanoseconds
    uint64_t currentTimeStep; // nanoseconds -- logged at the beginning of motor control loop function
    uint64_t prevTimeStep;    // nanoseconds -- logged immediately after a pulse has happened
    uint64_t limTimeStep;     // nanoseconds -- logged immediately after limit switch is triggered, used for debounce time
    uint64_t accTime;         // nanoseconds -- logged when acceleration added to speed in acceleration funciton (external)


    uint64_t nanos();

public:
    // RPI hardware configuration
    uint8_t stepPin; // needs to be pulsed between HIGH and LOW to achieve motion
    uint8_t dirPin;  // HIGH = Drives INWARD (towards stepper motor) || LOW = Drives OUTWARD (away from stepper motor)

    uint8_t limitInside;  // needs to be pulsed between HIGH and LOW to achieve motion
    uint8_t limitOutside; // HIGH = Drives INWARD (towards stepper motor) || LOW = Drives OUTWARD (away from stepper motor)

    // Current Motor Characteristics
    bool phase;      // phase of stepper motor (High 1 or Low 0) to power coils during actuation
    bool switchPress; // whether any switches are actively pressed (regardless of debounce logic) -- used during debounce determination in control loop
    uint32_t currentDelay; // nanoseconds -- this indicates the speed of stepper motor and is updated with setSpeed function (must be between Max and MinDelay)
    uint16_t stepCount;    // microsteps -- configured automatically during calibration by using limit switch as physical reference, updated every time step is pulsed
    int8_t motorDir;       // 1 for inward velocity  ||  -1 for outward velocity
    int8_t driveState;     // 0 = both hit | 1 = drive | 2 = inner drive | -1 = outer drive

    float currentSpeed; // -160 to 160 set by setSpeed() (mm/s)
    float goalSpeed;    // 0 to 160 set by setSpeedMagnitude() (mm/s)
    float acceleration; // mm/s/ms (determined by acc interval, which is currently set to 1 ms)

    uint16_t goalStepPosition;
    int32_t goalSteps;
    uint16_t remainingSteps;
    uint16_t accSteps;

    bool findingTarget; // boolean for if motor is actively seeking a goal position

    // Constructors
    MotorConfig(char Side);
    MotorConfig(uint8_t s, uint8_t d, uint8_t limOut, uint8_t limIn);
    MotorConfig();

    // SET FUNCTIONS
    void setHardware(uint8_t s, uint8_t d, uint8_t limOut, uint8_t limIn);

    void setStepPosition(uint16_t steps);

    void setPulseDelay(uint32_t delay); // directly change current pulse delay in ns

    void setSpeed(float speed); // use value from -100 to 100 to represent % speed -- deadband -1% to +1% -- see motorConfig.pdf for speed charts
    
    void setSpeedMagnitude(float speed);
    
    void setAcceleration(float acc);
    
    void setGoalPosition(uint16_t position, float speed);

    // MOTOR CONTROL FUNCTIONS
    void moveIn(uint64_t delay); // no limit sw monitoring
    
    void moveOut(uint64_t delay); // no limit sw monitoring
    
    void motorDrive(); // TO BE USED IN CONTROL LOOP -- decides when its time to step the motor based on setSpeed, keeps track of position as well
    void controlLoop(); // TO BE USED IN ROS Motor Control NODE -- update at 1GHz->2GHz

    // Goal and acceleration functions
    
    void accToSpeed(float speed); // ACCELERATION SETSPEED() ALTERNATIVE -- use in ROS loop which subscribes to commanded speed, true derivative of velocity
    
    void goalPosition_OLD(uint16_t goalPos, float speed); // for multiple motors at a time -- use return value to see when position is achieved
    
    void goalPosition(); // for multiple motors at a time -- use return value to see when position is achieved
};

#endif // MOTORCONFIG_H