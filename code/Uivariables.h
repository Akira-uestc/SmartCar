#ifndef _UNVARIABLES_H_
#define _UNVARIABLES_H_

#include <Platform_Types.h>
 /*********************************************************标志位*********************************************************/
 //全局
extern volatile int   Flag_Stop;
//出入库
extern volatile short  Flag_Garage_Out;
extern volatile short  Flag_Garage_Turn_Dircection;

/**********************************************************舵机***********************************************************/
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
/*********************************************************速度*********************************************************/
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

/********************************************************* T字路 *********************************************************/
extern volatile short  Ui_TSpeed_Change;


/*********************************************************时间*********************************************************/


/********************************************************* 出入库 *********************************************************/
extern volatile short Ui_Garage_Out_Turn_Line;
extern volatile short Ui_Garage_Out_Turn_Time;
extern volatile short Ui_Garage_In_Turn_Line;  
extern volatile short Ui_Garage_In_Turn_Time;

#endif
