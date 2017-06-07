#include "MotorPid.h"

MotorPID::MotorPID()
{
}


MotorPID::~MotorPID()
{
}

void MotorPID::Setup(float kp, float ki, float kd)
{
	Kp = kp;
	Ki = ki;
	Kd = kd;
}

float MotorPID::Process(float Target, float Measured_value)
{
	//P = Power of correction
	//I = Time correction
	//D = Future prodiction
	uint32_t time = millis();
	if (LastTime == 0)
		LastTime = time;

	uint32_t dt = time - LastTime;
	LastTime = time;

	float error = Target - Measured_value;
	Integral = Integral + error * dt;

	if (Integral > 1000)
		Integral = 1000;
	else if (Integral < -1000)
		Integral = -1000;

	float derivative = (error - Previous_error) / dt;
	Previous_error = error;
	return (Kp * dt / 1000.0) * error + Ki * Integral + Kd * derivative;
}
