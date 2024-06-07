#ifndef ENDEFFECTORCONFIG_H
#define ENDEFFECTORCONFIG_H


#include <atomic>
#include <thread>

#include <stdint.h>     // For uint8_t, uint16_t, uint32_t, uint64_t
#include "MotorConfig.h"// For MotorConfig class and functions



class EndEffectorConfig 
{
    private:

        const float stepPerRev = 3200.0;
        const float mmPerRev = 25.0;
        const float linkLength = 350.0;
        const float carriageOffset = (93.5 + 30.0 + 56.5); // inside edge of aluminum bloc to center of macron rail + aluminum block thickness + carriage pin to inside edge of carriage
        const float endOffset = 130.0; // center of end effector to outer linkage pin center
        const float linearStage = 250.0; // mm of travel possible for linear actuators

        float MAX_X_SPEED = 25.0; // mm/s -- maximum x speed (+ or -) during the directionalDrive() function
        float MOTOR_SPEED_MAX = 140.0; // mm/s -- maximum motor speed during the directionalDrive() function

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

        float xCord = 0;
        float zCord = 0;

        bool motorMoving = false;

        //float xDirection; // x component of direction vector (direction vector = vector of magnitude 1.0)
        //float zDirection; // z component of direction vector (direction vector = vector of magnitude 1.0)

        //float baseLength; // length between outer pins on carriages
        //float basePartial; // for right triangle of hypotenuse link length
        
        bool calibrationSuccess; // determines wheteher robot needs calibrated or not

        // Instance Modifiers
        EndEffectorConfig(int left, int right); // instance modifier

        // set functions
        void setGlobalAcceleration(float acc); // sets both left and right acceleration values

        // position update functions
        void updateMotorPosition(); // updates motor position in mm
        void updateCurrentPosition(); // updates end effector position in mm

        // movement fucntions
        void goToPosition(float xCord, float zCord, float speed); // brings end effector to position specified (built in safety)

        void calibrateZero(float speed); // calibrates motor position

        void moveInX(float speed); // relative to robot, moves along x axis only

        void moveInZ(float speed); // relative to robot, moves along z axis only

        void directionalDrive(float CMD_X_SPEED, float CMD_Z_SPEED); // relative to end effector, will actuate end effector at cmd speeds , which are limited

        void directionalDrive_MAG(float CMD_X_HEADING, float CMD_Z_HEADING, float speed); // drives at heading specified (unit vector) and applies speed proportionally to faster motor
};

#endif // ENDEFFECTORCONFIG_H