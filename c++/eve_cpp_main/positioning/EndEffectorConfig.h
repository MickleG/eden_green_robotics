#ifndef ENDEFFECTORCONFIG_H
#define ENDEFFECTORCONFIG_H

#include <stdint.h>
#include <wiringPi.h>
#include <time.h>
#include <iostream>
#include <cmath>
#include "MOTORCONFIG.h"

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

        float maxAbs(float x, float y);

        uint64_t nanos();

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

        bool calibrationSuccess; // determines wheteher robot needs calibrated or not

        // Instance Modifiers
        EndEffectorConfig(int left, int right); // instance modifier

        // set functions
        void setGlobalAcceleration(float acc);

        // position update functions
        void updateMotorPosition();
        void updateCurrentPosition();

        // movement fucntions
        void goToPosition(float xCord, float zCord, float speed);

        void calibrateZero(float speed);

        void moveInX(float speed);

        void moveInZ(float speed);

        void directionalDrive(float CMD_X_SPEED, float CMD_Z_SPEED);

        void directionalDrive_MAG(float CMD_X_HEADING, float CMD_Z_HEADING, float speed);
};

#endif // ENDEFFECTORCONFIG_H