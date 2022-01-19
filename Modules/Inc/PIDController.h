/*
 * PIDController.h
 *
 *  Created on: Jun 10, 2021
 *      Author: raing
 */

#ifndef SRC_PIDCONTROLLER_H_
#define SRC_PIDCONTROLLER_H_
#include <stdint.h>

#define CycleDurationInmSec 50

class PIDController
{
public:
	PIDController();
	PIDController(float newKi, float newKp, float newKd, float newSP);
	int updateCycle(float input);
	float getOutput();
	float setKi(float newKi);
	float setKp(float newKp);
	float setKd(float newKd);
	float setSP(float newSP);

	virtual ~PIDController();
private:
	float Ki;
	float Kp;
	float Kd;
	float Output;
	float PreviousError;
	float CurrentError;
	float PreviousIntegral;
	float SetPoint;
	uint32_t PreviousTimestamp;
};
//int updateCycle(float input);
//float getOutput();
#endif /* SRC_PIDCONTROLLER_H_ */
