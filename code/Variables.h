#ifndef _VARIABLES_H_
#define _VARIABLES_H_

#include "Platform_Types.h"
#include "PID.h"


/****************PID结构体******************/
extern volatile _pid_param_t  LSpeed_pid;          //左电机控制PID
extern volatile _pid_param_t  RSpeed_pid;          //右电机控制PID

extern volatile _pid_param_t  Steer_pid;           //舵机控制PID

/****************电机、舵机、编码器******************/
extern volatile sint16 LSpeed_Duty;               //左电机设置占空比
extern volatile sint16 RSpeed_Duty;               //右电机设置占空比

extern volatile uint16 Servo_Duty;                //舵机设置占空比

extern volatile sint16 LEnc_Val;                  //编码器获得左轮速度
extern volatile sint16 REnc_Val;                  //编码器获得右轮速度

extern volatile float Enc_Val;                   //里程

extern volatile float Purpost_Speed;               //目标速度    （以EncSpeed为单位）
extern volatile short Purpost_Left_Speed;          //目标左轮速度（以EncSpeed为单位）
extern volatile short Purpost_Rigt_Speed;          //目标右轮速度（以EncSpeed为单位）

#endif