/*
 * LT_Motor_Ctrl.h
 *
 *  Created on: 2022年4月20日
 *      Author: logX
 */

#ifndef _LT_MOTOR_CTRL_H_
#define _LT_MOTOR_CTRL_H_

#include "stdlib.h"
#include "Platform_Types.h"
#include "PID.h"
#include "Variables.h"
#include "Uivariables.h"
#include <Function.h>
#include <Duty.h>
#include "ImageProcess.h"

void maincontrol(void);
/************************************************ 速度 ************************************************/
void Update_Purpost_Speed(void);
void ChangeSpeed(sint16 alpha);
void UnchangeSpeed(sint16 alpha);
void SetToFixedSpeed(short SetFixed_Speed);
void CancelSetToFixedSpeed(void);
void LongStriaghtRoad(float LoseCenterPoint_Cnt_f);
/************************************************ 角度 ************************************************/
void Update_Angle(short pattern, int angle);
void Update_PID(short pattern, int line);


#endif
