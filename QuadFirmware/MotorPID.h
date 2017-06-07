// PID.h

#ifndef _MotorPID_H
#define _MotorPID_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

class MotorPID
{
public:
	MotorPID();
	~MotorPID();

	void Setup(float kp, float ki, float kd);
	float Process(float Target, float Measured_value);
private:
	float Previous_error;
	float Integral;
	float LastTime;



	float Kp;
	float Ki;
	float Kd;
	//float freq = 30;
	//float Kp = 10 / freq;
	//float Ki = 0.0005;
	//float Kd = 0.005;

};
#endif

