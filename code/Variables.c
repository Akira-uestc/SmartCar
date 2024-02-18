#include <Variables.h>
#include "PID.h"

//电机频率
#define MOTOR_FREQUENCY    10000

/****************PID结构体******************/
volatile _pid_param_t  LSpeed_pid;          //左电机控制PID
volatile _pid_param_t  RSpeed_pid;          //右电机控制PID

volatile _pid_param_t  Steer_pid;           //舵机控制PID

/****************电机、舵机、编码器******************/
volatile sint16 LSpeed_Duty;               //左电机设置占空比
volatile sint16 RSpeed_Duty;               //右电机设置占空比

volatile uint16 Servo_Duty;                //舵机设置占空比

volatile sint16 LEnc_Val;                  //编码器获得左轮速度
volatile sint16 REnc_Val;                  //编码器获得右轮速度

volatile float Enc_Val;                   //里程

volatile float Purpost_Speed;               //目标速度    （以EncSpeed为单位）
volatile short Purpost_Left_Speed;          //目标左轮速度（以EncSpeed为单位）
volatile short Purpost_Rigt_Speed;          //目标右轮速度（以EncSpeed为单位）

