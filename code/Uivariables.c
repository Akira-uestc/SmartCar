#include <Uivariables.h>

 /*********************************************************标志位*********************************************************/
 //全局
volatile int    Flag_Stop = 0;                     //停车标志位

//出入库
volatile short  Flag_Garage_Out = 0;               //是否执行出库程序 （1执行，0不执行）
volatile short  Flag_Garage_Turn_Dircection = 0;   //出入库方向始终相同（1右转，0左转）

/**********************************************************舵机***********************************************************/
volatile short  Ui_Servo_Mid = 756;                //舵机中值对应的数值占空比
volatile short  Ui_Servo_Interval = 68;            //舵机左右转动的阈值
volatile float  Ui_Steer_P = 1.63f;                //舵机P
volatile float  Ui_Steer_D = 5.6f;                 //舵机D
volatile short  Ui_Control_Line = 26;			   //舵机识别行

//各元素PD
volatile float  Ui_Ring_P = 1.63f;                  //圆环舵机P
volatile float  Ui_Ring_D = 4.5f;                  //圆环舵机D
volatile float  Ui_Fork_P = 1.63f;                 //三岔舵机P
volatile float  Ui_Fork_D = 4.5f;                  //三岔舵机D
/*********************************************************电机*********************************************************/
volatile float  Ui_LMotor_P = 200.0f;              //左电机P参数
volatile float  Ui_LMotor_I = 15.0f;               //左电机I参数
volatile float  Ui_LMotor_D = 8.0f;                //左电机D参数
volatile float  Ui_RMotor_P = 200.0f;              //右电机P参数
volatile float  Ui_RMotor_I = 15.0f;               //右电机I参数
volatile float  Ui_RMotor_D = 8.0f;                //右电机D参数
volatile float  Ui_Motor_Max_Out = 9000;           //电机PID输出限幅
volatile uint8  Ui_PID_Error_Thr = 40;             //对误差阈值（使用bang - bang）
/*********************************************************速度*********************************************************/
//差速
volatile float  Ui_Straight_Diff_K = 0.140f;       //直道差速时减速比（单位：MotorDuty / ServoDuty）
volatile float  Ui_Bend_Diff_K = 0.105f;           //弯道差速时减速比（单位：MotorDuty / ServoDuty）
volatile float  Ui_Fixed_Diff_K = 0.28f;           //固定速度的减速比（单位：MotorDuty / ServoDuty）
//速度变化
volatile short  Ui_Max_Speed = 80;                 //正常跑时最大速度（单位：EncSpeed）
volatile short  Ui_Min_Speed = 70;                 //正常跑时最小速度（单位：EncSpeed）
volatile short  Ui_Center_Lost_Speed = 70;         //丢中线时用的速度（单位：EncSpeed）
volatile short  Ui_Inner_Speed = 60;               //转向内轮速度    （单位：EncSpeed）
volatile short  Ui_Ring_Speed = 75;                //跑圆环时速度    （单位：EncSpeed）
volatile short  Ui_Fork_Speed = 70;                //进出三叉速度    （单位：EncSpeed）
volatile short  Ui_ForkRoad_Speed = 75;            //跑三叉时速度    （单位：EncSpeed）
volatile short  Ui_Stop_Speed = 50;                //进入车库速度    （单位：EncSpeed）



/********************************************************* 坡道 *********************************************************/
/********************************************************* T字路 *********************************************************/
volatile short  Ui_TSpeed_Change = 50;

/********************************************************* 出入库 *********************************************************/
volatile short Ui_Garage_Out_Turn_Line = 30;       //什么时候出库打角
volatile short Ui_Garage_Out_Turn_Time = 0.4;      //出库打角多少秒
volatile short Ui_Garage_In_Turn_Line = 50;       //什么时候入库打角
volatile short Ui_Garage_In_Turn_Time = 0.4;       //入库打角多少秒



