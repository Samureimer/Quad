// Structs.h

#ifndef _Structs_H
#define _Structs_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

enum Statuses : byte
{
	None,
	Motor,
	AngleAndCorrection,
	Battery,

};

struct Command
{
	float Roll, Pitch, Power;
	Statuses StatusToSend;
};

//A package for the NRF24l01 can have a max length of 32 bytes
struct MotorStatus
{
	byte type = Motor;
	byte filler1;
	uint16_t filler2;
	float
		MotorFR,
		MotorFL,
		MotorRR,
		MotorRL,
		Power;
};

struct AngleAndCorrectionStatus
{
	byte type = AngleAndCorrection;
	byte filler1;
	uint16_t filler2;
	float
		Roll,
		Pitch,
		PowerPitch,
		PowerRoll;
};

struct BatteryStatus
{
	byte type = Battery;
	byte filler1;
	uint16_t filler2;
	float BatteryLimitationFactor;
	float BatteryVoltage;
};

#endif

