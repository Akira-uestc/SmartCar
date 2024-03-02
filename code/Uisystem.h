#ifndef _UISYSTEM_H_
#define _UISYSTEM_H_


#include <IfxGtm_PinMap.h>
#include <Encoder.h>
#include <stdint.h>
#include <IfxGtm_PinMap.h>
#include <stdio.h>
#include <_Impl/IfxPort_cfg.h>
#include "Variables.h"
#include <Uivariables.h>
#include "PID.h"

#include <zf_driver_gpio.h>
#include <zf_device_oled.h>
#include <zf_driver_delay.h>

//定义模块号
typedef enum
{
    KEY1 = 0,  //母板上按键0
    KEY2 = 1,
    KEY3 = 2,
    KEY4 = 3,    //母板上拨码开关0
    KEY5 = 4,
    KEY6 = 5,
} KEYn_e;


//定义的管脚要对应实际按键
#define KEY1p      P33_11  
#define KEY2p      P33_9 
#define KEY3p      P22_0 
#define KEY4p      P22_1
#define KEY5p      P23_1  
#define KEY6p      P22_2 


unsigned char KEY_Read(KEYn_e KEYno);
void L_MotorUiSystem(void);
void R_MotorUiSystem(void);
void SteerUiSystem(void);
void Fork_UiSystem(void);
void Ring_UiSystem(void);
void T_UiSystem(void);
void Speed_UiSystem(void);
void Ui_System(void);

#endif
