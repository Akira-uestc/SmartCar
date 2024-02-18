#ifndef _PID_H_
#define _PID_H_

#include <Platform_Types.h>

typedef struct
{
    float       kp;                         //P
    float       ki;                         //I
    float       kd;                         //D

    float       pid_out_p;                  //KP输出
    float       pid_out_i;                  //KI输出
    float       pid_out_d;                  //KD输出
    float       pid_out_increment;          //增量式PID输出
    float       pid_out;                    //pid输出

    float       current_error;              //当前偏差
    float       last_error;                 //上一次的偏差值
    float       far_error;                  //前两次的偏差值
    float       error_m;                    //误差阈值

    short       line;                       //识别行

}_pid_param_t;

/*
float Incremental_PID(volatile _pid_param_t * pid_param, float error, float maxout);
float Servo_PID(volatile _pid_param_t * pid_param, float maxout);
*/

#endif
