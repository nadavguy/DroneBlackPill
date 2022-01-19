/*
 * ControlAgent.cpp
 *
 *  Created on: Jun 10, 2021
 *      Author: raing
 */

#include "main.h"
#include "PIDController.h"
//#include "KalmanFilter.h"

PIDController pitchPID(0, 2, 0, 0);
PIDController rollPID(0, 2, 0, 0);

//KalmanFilter pitchKF(0.125, 8, 45, 0);
//KalmanFilter rollKF(0.125, 8, 45, 0);

uint32_t lastPWMUpdate = 0;

void initPIDs(void)
{

}

void updatePIDs(void)
{
//	chasisIMUAHRS.filteredPitch = pitchKF.getFilteredValue(chasisIMUAHRS.Pitch);
//	chasisIMUAHRS.filteredRoll = rollKF.getFilteredValue(chasisIMUAHRS.Roll);
//	pitchPID.updateCycle(chasisIMUAHRS.filteredPitch); //chasisIMUAHRS.Pitch
//	rollPID.updateCycle(chasisIMUAHRS.filteredRoll); //chasisIMUAHRS.Roll
//	if (HAL_GetTick() - lastPWMUpdate >= CycleDurationInmSec)
//	{
//		float rollOutput = rollPID.getOutput();
//		float pitchOutput = pitchPID.getOutput();
//		uint32_t currentPWMRoll = TIM2->CCR1;
//		uint32_t newPWMRoll = (uint32_t)(currentPWMRoll + (int) (rollOutput));
//		if (newPWMRoll < 750)
//		{
//			newPWMRoll = 750;
//		}
//		else if (newPWMRoll > 2250)
//		{
//			newPWMRoll = 2250;
//		}
//
//		uint32_t currentPWMPitch = TIM2->CCR2;
//		uint32_t newPWMPitch = (uint32_t)(currentPWMPitch + (int) (pitchOutput));
//		if (newPWMPitch < 750)
//		{
//			newPWMPitch = 750;
//		}
//		else if (newPWMPitch > 2250)
//		{
//			newPWMPitch = 2250;
//		}
//
//		updatePWM(chasisServo, newPWMRoll);
//		updatePWM(lidarServo, newPWMPitch);
//		lastPWMUpdate = HAL_GetTick();
//	}
}
