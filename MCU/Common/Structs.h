// PID.h

#ifndef _Structs_H
#define _Structs_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

struct Command
{
	float Roll, Pitch, Power;
	bool DoSendStatusreport;
};

struct Status
{
	//float ActualRoll, ActualPitch;
	float MotorFR, MotorFL, MotorRR, MotorRL;
	float Power, PowerPitch, PowerRoll;
	//float BatteryLimitationFactor;
	float BatteryVoltage;
};

#endif

