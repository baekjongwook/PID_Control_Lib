/*
 * PID_2022.c
 *
 *  Created on: 2022. 11. 1.
 *      Author: BAEK JONG WOOK
 */

#include "PID_2022.h"

void PID_Control(PID* dst, long target, long input)
{
	dst->nowValue = input;
	dst->target = target;

	dst->nowError = dst->nowValue - dst->target;
	dst->errorSum += dst->nowError;
	dst->errorDiff = dst->nowError - dst->pastError;

	if(dst->errorSumLimit != 0)
	{
		if(dst->errorSum > dst->errorSumLimit)
		{
			dst->errorSum = dst->errorSumLimit;
		}

		else if(dst->errorSum < -dst->errorSumLimit)
		{
			dst->errorSum = -dst->errorSumLimit;
		}
	}

	dst->nowOutput = dst->kP * dst->nowError + dst->kI * dst->errorSum + dst->kD * dst->errorDiff;

	if(dst->underOfPoint == 0)
	{
		return;
	}

	dst->nowOutput /= dst->underOfPoint;
	dst->pastError = dst->nowError;

	if(dst->outputLimit != 0)
	{
		if(dst->nowOutput > dst->outputLimit)
		{
			dst->nowOutput = dst->outputLimit;
		}

		else if(dst->nowOutput < -dst->outputLimit)
		{
			dst->nowOutput = -dst->outputLimit;
		}
	}
}

void Get_Motor_Status(MOTOR* dst, TIM_TypeDef* TIMx)
{
	dst->pastEn = dst->nowEn;
	dst->nowEn = TIMx->CNT;
	dst->m1 = dst->nowEn - dst->pastEn;

	if(dst->m1 < -60000)
	{
		dst->m1 += 65535;
	}
	else if(dst->m1 > 60000)
	{
		dst->m1 -= 65535;
	}

	dst->RPM = (60.0 * dst->m1) / (Tc * PPR);

	dst->Degree += (dst->m1 / (PPR / 360.0));
}
