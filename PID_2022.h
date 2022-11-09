/*
 * PID_2022.h
 *
 *  Created on: 2022. 11. 1.
 *      Author: BAEK JONG WOOK
 */

#ifndef PID_2022_H_
#define PID_2022_H_

#define Tc 0.01
#define PPR 8192.0

typedef struct _PID{
	long nowValue;
	long pastValue;

	long nowError;
	long pastError;
	long target;

	long errorSum;
	long errorSumLimit = 5000;
	long errorDiff;

	long nowOutput;
	long pastOutput;
	long outputLimit;

	long underOfPoint = 1000;

	long kP;
	long kI;
	long kD;
}PID;

typedef struct _MOTOR{
    long long nowEn = 0;
    long long pastEn = 0;
    long long m1 = 0;

    float RPM = 0;
    float Degree = 0;
}MOTOR;

void PID_Control(PID* dst, long target, long input);

void Get_Motor_Status(MOTOR* dst, TIM_TypeDef* TIMx);

#endif /* PID_2022_H_ */
