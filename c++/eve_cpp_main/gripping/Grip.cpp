#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0


#include <stdlib.h>
#include <stdio.h>
#include <cmath>

#include <unistd.h>  //used for delay

#include "dynamixel_sdk.h"
#include <Grip.h>

#define ADDR_PRO_MODEL 						0
#define ADDR_PRO_OPERATING_MODE 			11
#define ADDR_PRO_CURRENT_LIMIT 				38
#define ADDR_PRO_ACCELERATION_LIMIT			40
#define ADDR_PRO_VELOCITY_LIMIT 			44
#define ADDR_PRO_TORQUE_ENABLE 				64
#define ADDR_PRO_POSITION_D_GAIN			80
#define ADDR_PRO_POSITION_I_GAIN			82
#define ADDR_PRO_POSITION_P_GAIN			84
#define ADDR_PRO_FEEDFORWARD_2nd_GAIN		88
#define ADDR_PRO_FEEDFORWARD_1st_GAIN		90
#define ADDR_PRO_GOAL_CURRENT				102
#define ADDR_PRO_GOAL_VELOCITY				104
#define ADDR_PRO_PROFILE_ACCELERATION		108
#define ADDR_PRO_PROFILE_VELOCITY			112
#define ADDR_PRO_GOAL_POSITION				116
#define ADDR_PRO_MOVING						122
#define ADDR_PRO_MOVING_STATUS				123
#define ADDR_PRO_PRESENT_CURRENT			126
#define ADDR_PRO_PRESENT_POSITION			132

#define ADDR_PRO_DRIVE_MODE						10
#define ADDR_PRO_PROFILE_ACCELERATION_TIME		108
#define ADDR_PRO_PROFILE_TIME_SPAN				112

//Protocol Version
#define PROTOCOL_VERSION					2.0

//Default setting
#define BAUDRATE							57600
#define DEVICENAME							"/dev/ttyUSB0"  
#define TORQUE_ENABLE						1
#define TORQUE_DISABLE						0
#define DXL_MINIMUN_POSITION_VALUE			0.0
#define DXL_MAXIMUN_POSITION_VALUE			4095.0
#define DXL_MOVING_STATUD_THRESHOLD		 	10

//Motor's limits
#define VELOCITY_LIMIT						500				
#define ACCELERATION_LIMIT					32767
#define CURRENT_LIMIT						1193				//mA
#define MIN_MOTOR_ANGLE						0				//Use for mapping				
#define MAX_MOTOR_ANGLE						4095				//Use for mapping

//udpate 2019/02/08: switch motors control to time_based control / function trajectory generation depreciated
// setprofile(acc, velo) function can be rename
#define VELOCITY_BASED						0
#define TIME_BASED							4

class MotorXM430
{
private:
	//Motors
	uint8_t m_ID;
	uint32_t m_model_number;
	//Control
	uint8_t m_mode;
	int8_t m_operating_mode;
	uint8_t m_drivingmode;
	//Status
	uint32_t m_present_position;
	//Communication instance and variables
	dynamixel::PacketHandler* packetHandler;
	dynamixel::PortHandler* portHandler;
	int dxl_comm_result;
	uint8_t dxl_error;

	uint16_t minAngle;
	uint16_t maxAngle;
	
public:

	int m_current_limit;
	int m_goal_current;

	bool grabSuccess = true;
	//Constructor:
	MotorXM430(int ID, int operating_mode, int current_limit, int goal_current, uint16_t minAng, uint16_t maxAng);
	
	//Functions:
	//GetID: return the motors ID. Input: Void. Output : int ID. Example: MotorXM430 motor; printf("ID: %d\n",motor.GetID());
	int GetID();
	//GetModelNumber(): return the model of the motor. (ex: model numer 1020 is the MX430). This function is used to read the current of the motors. 
	//As the first function call in the construstor, we used this function to check if the motor is turn ON or without failure. In case of failure, this function will terminate the program.
	uint16_t GetModelNumber();
	//IsMoving(): return the moving status of the motor.Used for motor synchronisation when several motors are moving.
	uint8_t IsMoving();
	//MovingStatus: printf the moving status of the motors. Used for debugging.
	void MovingStatus();
	//ReadCurrent(): return the signed current of the motor in motorUnit. 
	int16_t ReadCurrent();
	
	//MAP: return the mapped angle value into the other unit. Used for converting angle degree to angle motors.]
	//Inputs: angle is the value we want to convert. in_min/max: the minimal/maximal angle value in the 1fst unit. out_min/max: the minimal/maximal angle value in the 2nd unit.
	//Output: the mapped angle.
	//Example: MAP(m_present_position, MIN_MOTOR_ANGLE, MAX_MOTOR_ANGLE, 0.0, 360.0)
	float MAP(uint32_t angle, long in_min, long in_max, long out_min, long out_max);
	
	//TorqueON - TorqueOFF: (dis-)activate the torque inside the motor. Without torque the motor will not move.
	void TorqueON();
	void TorqueOFF();

	//ReadAngle: return the current angle of the motor in degree
	float ReadAngle(); 
	
	//Goto: set the goal position in the motor. The function do not currently check the validity of the input
	//Input: wanted position in degree float
	void Goto(float position);
	
	//SetOperatingMode: set the operating mode in the motor (Availiable mode: 0 (Current), 1 (Velocity), 3 (Position), 4 (Extended position), 5 (current-based position), 16 (PWM))
	//Input: wanted mode uint8_t
	void SetOperatingMode(uint8_t mode);						
	//PrintOperationMode: printf in the console the operationmode. Used for debugging.
	void PrintOperatingMode();
	
	//SetPID: set the different parameters of the internal motor's PID controler.
	//Input the P / I / D value between 0-16383 uint16_t
	void SetPID(uint16_t P_gain, uint16_t I_gain, uint16_t D_gain);
	//PrintPID: printf the gain value of the motor's PID in the consol. Used for debugging.
	void PrintPID();
	
	//SetFFGain: set the different parameters of the internal motor's FeedForward controler.
	//Input the FF1 FF2 value between 0-16383 uint16_t
	void SetFFGain(uint16_t FF1Gain, uint16_t FF2Gain);
	//PrintFFGain: printf the gain value of the motor's FeedForward control in the consol. Used for debugging.
	void PrintFFGain();
	
	//SetProfile: set the acceleration and velocity profile of the motor. Used to tunned the motor behaviour and for motor synchronisation when several motors are moving.
	//Input: wanted velocity (RPM) and acceleration (Rev/min2) uint32_t. The function check if the wanted value are in the motor's limits ( XM430: VELOCITY_LIMIT 500 / ACCELERATION_LIMIT 32767)
	void SetProfile(uint32_t Velocity, uint32_t Acceleration);
	//PrintProfile: printf the Velocity and Acceleration value in the consol. Used for debugging.
	void PrintProfile();
	
	//SetCurrentLimit: set the maximun current (torque) output of the motor. Used for current-based position control (gripper)
	//Input is set in the define section CURRENT_LIMIT 1193
	void SetCurrentLimit();
	//PrintCurrentLimit: printf the value of the current limit (mA) in the console. Used for debugging.
	void PrintCurrentLimit();
	
	//SetGoalCurrent: set the goal current. Used for current-based position control
	//Input: wanted goal current mA uint16_t. The function check if the wanted goalcurrent is not exceeding the currentlimit
	void SetGoalCurrent(uint16_t GoalCurrent);
	//PrintGoalcurrent: printf the value of the goalcurrent (mA) in the console. Used for debugging.
	void PrintGoalCurrent();	
	
	//## Update 2019/02/08
	//SetDriving: set the driving mode of the motor, between TIMED_BASED and VELOCITY_BASED driving mode.
	//Input: driving mode input (0: Velocity / 4: Time)
	void SetDrivingMode(uint8_t type);
	//PrintDrivingMode: print and return the current driving mode of the motor
	uint8_t PrintDrivingMode();	
	//SetTimeProfile: set the acceleration profile of the motor. Ta is the acceleration time, and tf is the final time. 
	//Input: Ta, Tf in ms. If Ta = 0, the profile is rectangle / if Ta=0.5Tf, the profile is triangle. Please refer to Dynamixel documentation for more information
	void SetTimeProfile(uint32_t Ta, uint32_t Tf);
	//PrintTimeProfile: print the current Ta and Tf profile parameter of the motor
	void PrintTimeProfile();
};

int MotorXM430::GetID(){return m_ID;}

//Updated 2019/02/08
void MotorXM430::SetDrivingMode(uint8_t type)
{
	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, m_ID, ADDR_PRO_DRIVE_MODE, type);
}

uint8_t MotorXM430::PrintDrivingMode()
{
	uint8_t type;
	dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, m_ID, ADDR_PRO_DRIVE_MODE, &type);
	printf("Motor %d is on driving mode: %d\n", m_ID, type);
	return(type);
}

void MotorXM430::SetTimeProfile(uint32_t Ta, uint32_t Tf)
{
	dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, m_ID, ADDR_PRO_PROFILE_ACCELERATION_TIME, Ta);
	dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, m_ID, ADDR_PRO_PROFILE_TIME_SPAN, Tf);
}


void MotorXM430::PrintTimeProfile()
{
	uint32_t Ti, Tf;
	dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, m_ID, ADDR_PRO_PROFILE_ACCELERATION_TIME, &Ti);
	dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, m_ID, ADDR_PRO_PROFILE_TIME_SPAN, &Tf);
	printf("Motor %d, current acceleration time: %d / timespan: %d\n", m_ID, Ti, Tf);
}

//End update
uint16_t MotorXM430::GetModelNumber()
{
	uint16_t modelNumber=0;
	dxl_comm_result=packetHandler->read2ByteTxRx(portHandler, m_ID, ADDR_PRO_MODEL, &modelNumber);
	if(dxl_comm_result != COMM_SUCCESS)
	{
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
		usleep(1000000);
		printf("\n\n#######\nNot able to communicate with the motors\n" );
		printf("Likely cause: no power / motor failure\n#######\n ");
		usleep(1000000);
		printf("Program exit()...\n");
		exit (EXIT_FAILURE);
	}
	return(modelNumber);
}

uint8_t MotorXM430::IsMoving()
{
	uint8_t moving;
	dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, m_ID, ADDR_PRO_MOVING, &moving);
	return (moving);
}

void MotorXM430::MovingStatus()
{
	uint8_t movingstatus=0;
	dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, m_ID, ADDR_PRO_MOVING_STATUS, &movingstatus);
	if(movingstatus > 48)
	{
		printf("Motor %d is in Trapezodal Profile\n", m_ID);
	}
	else if((movingstatus < 35) && (movingstatus > 20))
	{
		printf("Motor %d is in Triangular Profile\n", m_ID);
	}
	else if(movingstatus > 3)
	{
		printf("Motor %d is in Rectangular Profile\n", m_ID);
	}
	else if(movingstatus >= 0)
	{
		printf("Motor %d is in Step Mode (No Profile)", m_ID);
	}
	else
	{
		printf("Motor %d UNKNOWN Profile\n", m_ID);
	}
}

int16_t MotorXM430::ReadCurrent()
{
	uint16_t current = 0;
	dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, m_ID, ADDR_PRO_PRESENT_CURRENT, &current);
	int16_t currentS = current;
	//printf("Implicit conversion: %d\n",currentS);
	return(currentS);
}
		
float MotorXM430::MAP(uint32_t angle, long in_min, long in_max, long out_min, long out_max)
{
	return (((float)angle - in_min) * (out_max - out_min) / (in_max-in_min) + out_min);
}
		
void MotorXM430::TorqueON()
{
	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, m_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);

	if(dxl_comm_result!=COMM_SUCCESS)
	{
			printf("%s\n",packetHandler->getTxRxResult(dxl_comm_result));
	}
	else if (dxl_error !=0)
	{
			printf("%s\n",packetHandler->getRxPacketError(dxl_error));
	}
	else
	{
			//printf("Motor %d: TORQUE ENABLED\n", m_ID);
	}
}
		
void MotorXM430::TorqueOFF()
{
	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, m_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);

	if(dxl_comm_result!=COMM_SUCCESS)
	{
			printf("%s\n",packetHandler->getTxRxResult(dxl_comm_result));
	}
	else if (dxl_error !=0)
	{
			printf("%s\n",packetHandler->getRxPacketError(dxl_error));
	}
	else
	{
			//printf("Motor %d: TORQUE DISABLE\n", m_ID);
	}
}

float MotorXM430::ReadAngle()
{
	dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, m_ID, ADDR_PRO_PRESENT_POSITION, &m_present_position, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
	{
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	}
	else if (dxl_error != 0)
	{
		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	}
	return(MAP(m_present_position, MIN_MOTOR_ANGLE, MAX_MOTOR_ANGLE, 0.0, 360.0));
}
		
void MotorXM430::Goto(float position)
{
	position=(int)MAP(position, 0, 360, MIN_MOTOR_ANGLE, MAX_MOTOR_ANGLE);
	dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, m_ID, ADDR_PRO_GOAL_POSITION, position, &dxl_error);
}

void MotorXM430::SetOperatingMode(uint8_t mode)
{
	TorqueOFF();
	if( (mode==0) || (mode==1) || (mode==3) || (mode==4) || (mode==5) || (mode==16))
	{
		dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, m_ID, ADDR_PRO_OPERATING_MODE, mode, &dxl_error);
		if (dxl_comm_result != COMM_SUCCESS)
			{
				printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	    		}
	    		else if (dxl_error != 0)
	      		{
		       		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	       		}
		else
		{
			m_mode=mode;
		}
	}
	else
	{
		printf("Invalid Control Mode (Availiable mode: 0, 1, 3, 4, 5, 16)\n");
	}

}						
void MotorXM430::PrintOperatingMode()
{
	uint8_t mode=3;
	dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, m_ID, ADDR_PRO_OPERATING_MODE, &mode, &dxl_error);
	switch(mode)
	{
	case 0:
			printf("Operation Mode 0 : Current Control Mode\n");
			break;
	case 1:
			printf("Operation Mode 1 : Velocity Control Mode\n");
			break;
	case 3:
			printf("Operation Mode 3 : Position Control Mode\n");
			break;
	case 4:
			printf("Operation Mode 4: Extented Position Control Mode (Multi-turn)\n");
			break;
	case 5:
			printf("Operation Mode 5: Current-base Position Control Mode\n");
			break;
	case 16:
			printf("Operation Mode 16: PWM Control Mode\n");
			break;
	default:
			printf("Invalid Control Mode (Availiable mode: 0, 1, 3, 4, 5, 16)\n");
	}
}
		
void MotorXM430::SetPID(uint16_t P_gain, uint16_t I_gain, uint16_t D_gain)
{
	dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, m_ID, ADDR_PRO_POSITION_P_GAIN, P_gain, &dxl_error);
	dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, m_ID, ADDR_PRO_POSITION_I_GAIN, I_gain, &dxl_error);
	dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, m_ID, ADDR_PRO_POSITION_D_GAIN, D_gain, &dxl_error);
	// check if PID are set:
	PrintPID();
}
void MotorXM430::PrintPID()
{
	uint16_t P, I, D;
	dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, m_ID, ADDR_PRO_POSITION_P_GAIN, &P, &dxl_error);
	dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, m_ID, ADDR_PRO_POSITION_I_GAIN, &I, &dxl_error);
	dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, m_ID, ADDR_PRO_POSITION_D_GAIN, &D, &dxl_error);
	printf("Motor %d: 	PID values : P: %d / I: %d / D: %d\n", m_ID, P, I, D);
}

void MotorXM430::SetFFGain(uint16_t FF1Gain, uint16_t FF2Gain)
{
	dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, m_ID, ADDR_PRO_FEEDFORWARD_1st_GAIN, FF1Gain);
	dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, m_ID, ADDR_PRO_FEEDFORWARD_2nd_GAIN, FF2Gain);
	// check if FFgain are set:
	PrintFFGain();
}
void MotorXM430::PrintFFGain()
{
	uint16_t FF1Gain, FF2Gain;
	dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, m_ID, ADDR_PRO_FEEDFORWARD_1st_GAIN, &FF1Gain);
	dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, m_ID, ADDR_PRO_FEEDFORWARD_2nd_GAIN, &FF2Gain);
	printf("Motor %d, Feed forward Gain: FF1: %d / FF2: %d\n", m_ID, FF1Gain, FF2Gain);
}
		
void MotorXM430::SetProfile(uint32_t Velocity, uint32_t Acceleration)
{
	//Set the limits:
	dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, m_ID, ADDR_PRO_VELOCITY_LIMIT, VELOCITY_LIMIT);
	dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, m_ID, ADDR_PRO_ACCELERATION_LIMIT, ACCELERATION_LIMIT);
	if(Velocity<=VELOCITY_LIMIT)
	{
		dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, m_ID, ADDR_PRO_PROFILE_VELOCITY, Velocity);
	}
	else 
	{
		printf("Velocity out of range (limit=%d)\n",VELOCITY_LIMIT);
	}
	if(Acceleration<=ACCELERATION_LIMIT)
	{
		dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, m_ID, ADDR_PRO_PROFILE_ACCELERATION, Acceleration);
	}
	else
	{
		printf("Acceleration out of range (limit=%d)\n",ACCELERATION_LIMIT);
	}	
}
void MotorXM430::PrintProfile()
{
	uint32_t acceleration=0;
	uint32_t velocity=0;
	dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, m_ID, ADDR_PRO_PROFILE_VELOCITY, &velocity);
	dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, m_ID, ADDR_PRO_PROFILE_ACCELERATION, &acceleration);
	printf("Motor %d:	Acceleration Profile: %d and Velocity Profile: %d\n",m_ID, acceleration, velocity);
} 
		
void MotorXM430::SetGoalCurrent(uint16_t GoalCurrent)
{
	if((GoalCurrent>=-m_current_limit) || (GoalCurrent<=m_current_limit))
	{
		dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, m_ID, ADDR_PRO_GOAL_CURRENT, GoalCurrent);
	}
	else
	{
		printf("Goal current out of range (limit=%d)\n",m_current_limit);
	}
}
void MotorXM430::PrintGoalCurrent()
{
uint16_t goalCurrent=0;
dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, m_ID, ADDR_PRO_GOAL_CURRENT, &goalCurrent);
printf("Motor %d: Goal Current: %d mA\n", m_ID, goalCurrent);
}
			
void MotorXM430::SetCurrentLimit()
{
	dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, m_ID, ADDR_PRO_CURRENT_LIMIT, m_current_limit);
}
void MotorXM430::PrintCurrentLimit()
{
	float SetUnit = 1;
	uint16_t current_limit = CURRENT_LIMIT;
	float current_limit_amp =0;
	float UnitMX430 = 2.69;
	switch(m_model_number)
	{
		case 1020:
			SetUnit = UnitMX430;
			break;
			
		default:
			printf("Motor %d, model not supported.\n",m_ID);
	}
	dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, m_ID, ADDR_PRO_CURRENT_LIMIT, &current_limit);
	current_limit_amp=(float)current_limit*(SetUnit/1000.0);
	printf("Motor %d, Current limit: %d (input motor), %f (amp)\n", m_ID, current_limit, current_limit_amp);
}

//Specify to initialize members in the member initialiser list:
MotorXM430::MotorXM430(int ID, int operating_mode, int current_limit, int goal_current, uint16_t minAng, uint16_t maxAng):portHandler(dynamixel::PortHandler::getPortHandler(DEVICENAME)),packetHandler(dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION))
{	
	//Communication initialisation
	dxl_comm_result = COMM_TX_FAIL;  
	dxl_error=0;

	if (portHandler->openPort()){}

	else
	{
		printf("Motor %d: Failed to open the port!\n",m_ID);
	}

	if (portHandler->setBaudRate(BAUDRATE)){}

	else
	{
		printf("Motor %d: Failed to change the baudrate!\n",m_ID);
	}

	//Motor initialisation:
	m_ID = ID;
	m_model_number = GetModelNumber();
	m_mode = operating_mode;
	SetOperatingMode(m_mode);
	m_current_limit = current_limit;
	SetCurrentLimit();
	m_goal_current = goal_current;
	SetGoalCurrent(goal_current);
	m_present_position=ReadAngle();
	printf("UPDATE: drivingmode setting: TIME_BASED MODE\n");
	SetDrivingMode(TIME_BASED);
	m_drivingmode = PrintDrivingMode();
	printf("Motor %d initialized\n", m_ID);

	minAngle = minAng;
	minAngle = maxAng;

};

void drop(MotorXM430 servo1, MotorXM430 servo2)
{
	servo1.TorqueON();
	servo2.TorqueON();
	servo1.Goto(280);
	servo2.Goto(260);

}

void grip(MotorXM430 servo1, MotorXM430 servo2)
{
	float rightClosed = 225; // relative to eve
	float leftClosed = 315;
	float tolerance = 12.5; // defines grabSuccess boolean which can be published for state machine

	servo1.TorqueON();
	servo2.TorqueON();
	servo1.Goto(rightClosed);
	servo2.Goto(leftClosed);

	while(!(servo1.ReadAngle() <= rightClosed || servo2.ReadAngle() >= leftClosed) && !(abs(servo1.ReadCurrent()) >= servo1.m_current_limit || abs(servo2.ReadCurrent()) >= servo2.m_current_limit))
	{
		printf("GRIP ACTIVE\n");
		printf("Motor %d, current: %d\n",servo1.GetID(), abs(servo1.ReadCurrent()));
		printf("Motor %d, current: %d\n\n",servo2.GetID(), abs(servo2.ReadCurrent()));

	}

	if(servo1.ReadAngle() >= (rightClosed + tolerance) || servo2.ReadAngle() <= (315 - tolerance))
	{
		printf("\n\n  GRIP FAILED\n\n");
		printf("Motor %d, current position: %f\n",servo1.GetID(), servo1.ReadAngle());
		printf("Motor %d, current position: %f\n\n",servo2.GetID(), servo2.ReadAngle());

		servo1.grabSuccess = false;
		servo2.grabSuccess = false;

		printf("%d\n", servo1.grabSuccess);

	}

	else
	{
		printf("\n\n  GRIP COMPLETE\n\n");
		printf("Motor %d, current position: %f\n",servo1.GetID(), servo1.ReadAngle());
		printf("Motor %d, current position: %f\n\n",servo2.GetID(), servo2.ReadAngle());

		servo1.grabSuccess = true;
		servo2.grabSuccess = true;

		printf("%d\n", servo1.grabSuccess);
	}

}

int main()
{

	int curLimit = 90;
	int goalCur = 200;

	//Create a servo motor: servo*(dynamyxel ID, mode, currentlimit, goalcurrent, min angle, max angle)
	MotorXM430 servo1(1, 5, curLimit, goalCur, 225, 315);
	MotorXM430 servo2(2, 5, curLimit, goalCur, 315, 225);
	
	//Accessing the motors function
	servo1.PrintOperatingMode();
	servo2.PrintOperatingMode();

	servo1.SetProfile(32000, 400); // have to be in velocity mode I think
	servo2.SetProfile(32000, 400); // have to be in velocity mode I think
	
	//Controlling the motor

	usleep(500000);

	drop(servo1, servo2);

	usleep(500000);

	grip(servo1, servo2);

	usleep(5000000);

	servo1.TorqueOFF();
	servo2.TorqueOFF();
	
	usleep(100000);

	while (1)
	{
		printf("Motor %d, current position: %f\n",servo1.GetID(), servo1.ReadAngle());
		printf("Motor %d, current position: %f\n\n",servo2.GetID(), servo2.ReadAngle());
		
		usleep(250000);
	}
	
	
	//Adding others motor
	
	return(0);
}
