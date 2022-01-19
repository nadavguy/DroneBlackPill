/*
 * PIDController.cpp
 *
 *  Created on: Jun 10, 2021
 *      Author: raing
 */
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "PIDController.h"

PIDController::PIDController()
{
	// TODO Auto-generated constructor stub
	
}

PIDController::PIDController(float newKi, float newKp, float newKd, float newSP)
{
	Ki = newKi;
	Kp = newKp;
	Kd = newKd;
	SetPoint = newSP;
}

PIDController::~PIDController()
{
	// TODO Auto-generated destructor stub
}

int PIDController::updateCycle(float input)
{
	if (HAL_GetTick() - PreviousTimestamp >= CycleDurationInmSec)
	{
		PreviousTimestamp = HAL_GetTick();
		CurrentError = SetPoint - input;
		float Proportional = CurrentError;
		float Integral = PreviousIntegral + CurrentError * CycleDurationInmSec;
		float Derivative = (CurrentError - PreviousError) / CycleDurationInmSec;
		Output = Kp * Proportional + Ki * Integral + Kd * Derivative;
		PreviousError = CurrentError;
		PreviousIntegral = Integral;
		return 0;
	}
	return -1;
}

float PIDController::getOutput()
{
	return Output;
}

float PIDController::setKi(float newKi)
{
	Ki = newKi;
	return Ki;
}

float PIDController::setKp(float newKp)
{
	Kp = newKp;
	return Kp;
}

float PIDController::setKd(float newKd)
{
	Kd = newKd;
	return Kd;
}

float PIDController::setSP(float newSP)
{
	SetPoint = newSP;
	return SetPoint;
}
