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
        static const float stepPerRev;
        static const float mmPerRev;
        static const float linkLength;
        static const float carriageOffset;
        static const float endOffset;
        static const float linearStage;
        float MAX_X_SPEED;
        float MAX_Z_SPEED;

        float maxAbs(float x, float y);
        uint64_t nanos();

    public:
        MotorConfig leftMotor;
        MotorConfig rightMotor;

        float leftMotorPosition;
        float rightMotorPosition;
        float rightMotorSpeed;
        float leftMotorSpeed;
        float xPosition;
        float zPosition;
        float xDirection;
        float zDirection;
        float baseLength;
        float basePartial;
        float tempMax;
        bool calibrationSuccess;

        EndEffectorConfig(int left, int right);
        void setGlobalAcceleration(float acc);
        void updateMotorPosition();
        void updateCurrentPosition();
        void goToPosition(float xCord, float zCord, float speed);
        void calibrateZero(float speed);
        void moveInX(float speed);
        void moveInZ(float speed);
        void directionalDrive(float CMD_X_SPEED, float CMD_Z_SPEED);
        void directionalDrive_MAG(float CMD_X_HEADING, float CMD_Z_HEADING, float speed);
};

#endif // ENDEFFECTORCONFIG_H