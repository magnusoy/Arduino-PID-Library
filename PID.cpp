#include "PID.h"
#include "math.h"

PID::PID(double kp, double ki, double kd, int direction)
{
	this->direction = direction;
	PID::setParams(kp, ki, kd);
	lastUpdateTime = millis();
}

double PID::compute(double actual, double target)
{

	if (lastUpdateTime + updateTime <= millis())
	{
		double lastError = error;
		error = target - actual;
		double output = error * kp;
		outputSum += error * ki;
		double outputLast = (lastError - error) * kd;
		outputValue = offset + output + outputSum + outputLast;
		if (outputValue > outputLimitHigh)
		{
			outputValue = outputLimitHigh;
		}
		if (outputValue < outputLimitLow)
		{
			outputValue = outputLimitLow;
		}
		lastUpdateTime = millis();
	}
	return outputValue;
}

bool PID::reset()
{
	bool reset = false;
	lastUpdateTime = millis();
	error = 0.0f;
	outputSum = 0.0f;
	outputValue = 0.0f;
}

void PID::setLimits(double low, double high)
{
	this->outputLimitLow = low;
	this->outputLimitHigh = high;
}

void PID::setOffset(double offset) { this->offset = offset; }

void PID::setUpdateTime(unsigned long updateTime) { this->updateTime = updateTime; }

void PID::setParams(double kp, double ki, double kd)
{
	if (direction == 0)
	{
		this->kp = kp;
		this->ki = ki;
		this->kd = kd;
	}
	else if (direction == 1)
	{
		this->kp = -kp;
		this->ki = -ki;
		this->kd = -kd;
	}
}
