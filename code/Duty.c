#include <Duty.h>

volatile int CurrentServoDty;

/*
* @brief    设置舵机占空比
* @param    duty ：占空比*ATOM_PWM_MAX(10000.0f) （duty为数值占空比）
* @note     舵机中值 1.53ms高电平的PWM波    限幅在 1.36 - 1.7ms之间
*/
inline void Set_Servo_Duty(uint16 duty)
{
    duty = Constrain_Float((float)duty,(float)(Ui_Servo_Mid - Ui_Servo_Interval), (float)(Ui_Servo_Mid + Ui_Servo_Interval));
    CurrentServoDty = duty;
    pwm_init(ATOMSERVO, duty, 50);
}

/**
  * @brief   设置电机占空比
  * @param   duty ：占空比 * ATOM_PWM_MAX(10000.0f) （duty为数值占空比）
  * @note    num为0 修改右电机占空比；  num为1 修改左电机占空比
  * @date    2022.4.20
  */

inline void Set_Motor_Duty(int num, short duty)
{
    if(num)
    {
        if(duty >= 0)
        {
            pwm_init(MOTOR1_P, MOTOR_FREQUENCY, duty);
            IfxPort_setPinLow(&MODULE_P22, 3);
        }
        else
        {
            pwm_init(MOTOR1_P, MOTOR_FREQUENCY, duty + MOTOR_FREQUENCY);
            IfxPort_setPinHigh(&MODULE_P22, 3);
        }
    }

    else
    {
        if(duty >= 0)
        {
           pwm_init(MOTOR2_P, MOTOR_FREQUENCY, duty);
            IfxPort_setPinLow(&MODULE_P21, 3);
        }
        else
        {
            pwm_init(MOTOR2_P, MOTOR_FREQUENCY, duty + MOTOR_FREQUENCY);
            IfxPort_setPinHigh(&MODULE_P21, 3);
        }
    }
}
