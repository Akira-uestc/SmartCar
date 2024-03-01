#ifndef _DUTY_H_
#define _DUTY_H_

//舵机占空比
volatile int CurrentServoDty;

#include <Platform_Types.h>
#include <Function.h>
#include <stdint.h>
#include "Uivariables.h"
#include <zf_driver_pwm.h>
#include "Init.h"

#define ATOMSERVO    ATOM2_CH5_P33_13

#define MOTOR2_P      ATOM0_CH2_P21_4
#define MOTOR2_N      ATOM0_CH1_P21_3
#define MOTOR1_P      ATOM0_CH0_P21_2
#define MOTOR1_N      ATOM0_CH4_P22_3


extern volatile int CurrentServoDty;//当前的舵机占空比

void Set_Servo_Duty(uint16 duty);
void Set_Motor_Duty(int num, short duty);

#endif
