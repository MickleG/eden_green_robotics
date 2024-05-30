#include <stdio.h> 
#include <stdlib.h>
#include <wiringPi.h>   // Include WiringPi library!
#include <time.h>       // For NANOS function
#include <iostream>
#include <cmath>        // For sqrt and other math functions
#include <DynamixelSDK.h> // For servo motors
#include <ServoDrive.h>
#include <EndEffectorConfig.h>
#include <MotorConfig.h>

using namespace std;

class EveStateMachine 
{

	private:




	public:

		uint8_t state;

		bool limitCheck;
		bool motorCheck; // when encoders exist, we can use this variable by running a few steps on each motor and looking at encoder value
		bool realSenseCheck;
		bool dynamixelCheck;
		bool eStopped;
		bool calibrated;

		float leftMotorPosition; // mm from outside
		float rightMotorPosition; // mm from outside

		float xPosition; // mm from macron center
		float zPosition; // mm from fully folded
		float yPosition; // mm from bottom of vine

		bool vineDetected;
		bool scissorsRetracted;
		bool grippersClosed;

		uint8_t cutAttempts;

		EveStateMachine()
		{
			state = 1;

			limitCheck = false;
			realSenseCheck = false;
			dynamixelCheck = false;
			eStopped = false;
			calibrated = false;

			vineDetected = false;
			scissorsRetracted = true;
			grippersClosed = false;

			cutAttempts = 0;

		}
		void switchState()
		{

			switch (state)
			{
				case 1: // STARTUP

					if (limitCheck && realSenseCheck && dynamixelCheck && !eStopped)
					{
						state = 2;
					}

					else
					{
						// dynamixel check
						// realSense check
						// check limit sw
						// check eStop
					}

					break;


				case 2: // CALIBRATION

					break;


				case 3: // PAUSE 				... ayo

					break; // 


				case 4: // FIRST_Z_STEP

					break;


				case 5: // XZ_SERVOING

					break;


				case 6: // GRAB_CUT_PLANT

					break;


				case 7: // CUT_FAILED

					break;


				case 8: // 

					break;


				case 9:

					break;


				case 10:

					break;

			}
		}



}