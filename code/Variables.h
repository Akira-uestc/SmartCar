#ifndef _VARIABLES_H_
#define _VARIABLES_H_

#include "Platform_Types.h"
#include "PID.h"

 //全局
extern volatile int   Flag_Stop;
//出入库
extern volatile short  Flag_Garage_Out;
extern volatile short  Flag_Garage_Turn_Dircection;

//舵机
extern volatile short Ui_Servo_Mid;
extern volatile short Ui_Servo_Interval;
extern volatile float Ui_Steer_P;
extern volatile float Ui_Steer_D;
extern volatile short Ui_Control_Line;
//各元素PD
extern volatile float  Ui_Ring_P;
extern volatile float  Ui_Ring_D;
extern volatile float  Ui_Fork_P;
extern volatile float  Ui_Fork_D;
/*********************************************************电机*********************************************************/
extern volatile float Ui_LMotor_P;
extern volatile float Ui_LMotor_I;
extern volatile float Ui_LMotor_D;
extern volatile float Ui_RMotor_P;
extern volatile float Ui_RMotor_I;
extern volatile float Ui_RMotor_D;
extern volatile float Ui_Motor_Max_Out;
extern volatile uint8 Ui_PID_Error_Thr;

//速度
//差速
extern volatile float Ui_Fixed_Diff_K;
extern volatile float Ui_Straight_Diff_K;
extern volatile float Ui_Bend_Diff_K;
//速度变化
extern volatile short Ui_Max_Speed;
extern volatile short Ui_Min_Speed;
extern volatile short  Ui_Inner_Speed;
extern volatile short Ui_Center_Lost_Speed;
extern volatile short Ui_Ring_Speed;
extern volatile short Ui_ForkRoad_Speed;
extern volatile short Ui_Fork_Speed;
extern volatile short Ui_Stop_Speed;

//T字路
extern volatile short  Ui_TSpeed_Change;

//出入库
extern volatile short Ui_Garage_Out_Turn_Line;
extern volatile short Ui_Garage_Out_Turn_Time;
extern volatile short Ui_Garage_In_Turn_Line;  
extern volatile short Ui_Garage_In_Turn_Time;

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
