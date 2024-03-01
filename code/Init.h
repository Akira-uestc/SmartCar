#ifndef _INIT_H_
#define _INIT_H_

#include <zf_device_oled.h>
#include <zf_driver_gpio.h>
#include <zf_driver_exti.h>
#include <zf_driver_pit.h>
#include <zf_device_oled.h>

#include <Encoder.h>
#include "Variables.h"
#include "Uivariables.h"
#include "PID.h"
#include "Duty.h"

//电机频率
#define MOTOR_FREQUENCY    10000

void Motor_Init(void);
void Servo_Init(void);
void Screen_Init(void);
void GPIO_Init(void);
void Encoder_Init(void);
void CCU_Init(void);
void Motor_PID_Init(void);
void Steer_PID_Init(void);
void Init_All(void);
void Ui_Init(void);

#endif
