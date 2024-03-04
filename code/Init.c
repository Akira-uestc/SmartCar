#include <Init.h>

extern volatile _pid_param_t  LSpeed_pid;          //左电机控制PID
extern volatile _pid_param_t  RSpeed_pid;          //右电机控制PID
extern volatile _pid_param_t  Steer_pid;           //舵机控制PID



/*电机初始化*/
void Motor_Init(void)
{
    pwm_init(MOTOR1_P, MOTOR_FREQUENCY, 0);
    pwm_init(MOTOR2_P, MOTOR_FREQUENCY, 0);
    gpio_init(P21_3, 1, 0, GPO_PUSH_PULL);
    gpio_init(P22_3, 1, 0, GPO_PUSH_PULL);

}


/*舵机初始化*/
void Servo_Init(void)
{
    pwm_init(ATOMSERVO, 50, Ui_Servo_Mid);
}


/*OLED屏幕初始化（清屏）*/
void Screen_Init(void)
{
    //OLED初始化
    oled_init();
    //OLED屏幕清空
    oled_clear();
}


/*GPIO管脚初始化*/
void GPIO_Init(void)
{
    //蜂鸣器
    gpio_init(P20_3, 1, 0,GPO_PUSH_PULL);

    //按键
    gpio_init(P33_11, 0, 1,GPI_FLOATING_IN);
    gpio_init(P33_9, 0, 1,GPI_FLOATING_IN);
    gpio_init(P22_0, 0, 1,GPI_FLOATING_IN);
    gpio_init(P22_1, 0, 1,GPI_FLOATING_IN);
    gpio_init(P23_1, 0, 1,GPI_FLOATING_IN);
    gpio_init(P22_2, 0, 1,GPI_FLOATING_IN);

    /*
    //电源模块使能
    PIN_InitConfig(P00_7,PIN_MODE_OUTPUT,1);
    */
}


/*编码器初始化*/
void Encoder_Init(void)
{
    ENC_InitConfig(ENC2_InPut_P33_7,ENC2_Dir_P33_6);
    ENC_InitConfig(ENC5_InPut_P10_3,ENC5_Dir_P10_1);
}


/*CCU6初始化*/

void CCU_Init(void)
{
    pit_ms_init(CCU60_CH0, 5);  //5ms进行一次中断
    pit_enable(CCU60_CH0);

    pit_ms_init(CCU61_CH0, 3); //3ms进行一次中断
    pit_enable(CCU61_CH0);
}


/*PID参数初始化*/
void Motor_PID_Init(void)
{
    LSpeed_pid.kp                = Ui_LMotor_P;
    LSpeed_pid.ki                = Ui_LMotor_I;
    LSpeed_pid.kd                = Ui_LMotor_D;
    LSpeed_pid.pid_out_p         = 0;
    LSpeed_pid.pid_out_i         = 0;
    LSpeed_pid.pid_out_d         = 0;
    LSpeed_pid.pid_out_increment = 0;
    LSpeed_pid.pid_out           = 0;
    LSpeed_pid.current_error     = 0;
    LSpeed_pid.last_error        = 0;
    LSpeed_pid.far_error         = 0;
    LSpeed_pid.error_m           = Ui_PID_Error_Thr;

    RSpeed_pid.kp                = Ui_RMotor_P;
    RSpeed_pid.ki                = Ui_RMotor_I;
    RSpeed_pid.kd                = Ui_RMotor_D;
    RSpeed_pid.pid_out_p         = 0;
    RSpeed_pid.pid_out_i         = 0;
    RSpeed_pid.pid_out_d         = 0;
    RSpeed_pid.pid_out_increment = 0;
    RSpeed_pid.pid_out           = 0;
    RSpeed_pid.current_error     = 0;
    RSpeed_pid.last_error        = 0;
    RSpeed_pid.far_error         = 0;
    RSpeed_pid.error_m           = Ui_PID_Error_Thr;

}

void Steer_PID_Init(void)
{
    Steer_pid.kp                = Ui_Steer_P;
    Steer_pid.ki                = 0;
    Steer_pid.kd                = Ui_Steer_D;
    Steer_pid.pid_out_p         = 0;
    Steer_pid.pid_out_i         = 0;
    Steer_pid.pid_out_d         = 0;
    Steer_pid.pid_out_increment = 0;
    Steer_pid.pid_out           = 0;
    Steer_pid.current_error     = 0;
    Steer_pid.last_error        = 0;
    Steer_pid.far_error         = 0;
    Steer_pid.line              = Ui_Control_Line;
    Steer_pid.error_m           = Ui_PID_Error_Thr;
}

/*对所有模块进行初始化*/
void Init_All(void)
{
    //遥控器 上升沿触发中断
    //exti_init(ERU_CH5_REQ1_P15_8, EXTI_TRIGGER_RISING);
    Motor_Init();
    Servo_Init();
    Encoder_Init();
    CCU_Init();
    Screen_Init();
}

void Ui_Init(void)
{
    Flag_Stop = 0;
    Motor_PID_Init();
    Steer_PID_Init();

}

