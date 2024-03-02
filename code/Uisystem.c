#include <uisystem.h>


#pragma warning 544         // 屏蔽警告
 /*************************************************************************
 *  函数名称：unsigned char KEY_Read(KEYn_e KEYno)
 *  功能说明：读取按键状态
 *  参数说明：KEYn_e KEYno按键编号
 *  函数返回：按键状态，0/1
 *  修改时间：2020年3月10日
 *  备    注：
 *************************************************************************/
unsigned char KEY_Read(KEYn_e KEYno)
{
    switch (KEYno)
    {
    case KEY1:
        return gpio_get_level(KEY1p);//母板上按键0
        break;

    case KEY2:
        return gpio_get_level(KEY2p);//母板上按键1
        break;

    case KEY3:
        return gpio_get_level(KEY3p);//母板上按键2
        break;

    case KEY4:
        return gpio_get_level(KEY4p);//母板上按键2
        break;

    case KEY5:
        return gpio_get_level(KEY5p);//母板上拨码开关0
        break;

    case KEY6:
        return gpio_get_level(KEY6p);//母板上拨码开关1
        break;
    default:
        return 0XFF;
    }
    return 0;
}
#pragma warning default     // 打开警告
void R_MotorUiSystem(void)
{
    int Flag = 0, flag = 0, CFlag = 0, cflag = 0;//定义调参ui的标志数，flag表示选择层数，cflag表示是否选中并进行调整
    char txt[16];
    sprintf(txt, "P: %05f;   ", Ui_RMotor_P);
    oled_show_string(0, 0, txt);
    sprintf(txt, "I: %05f;   ", Ui_RMotor_I);
    oled_show_string(0, 1, txt);
    sprintf(txt, "D: %05f;   ", Ui_RMotor_D);
    oled_show_string(0, 2, txt);
    sprintf(txt, "T: %05d;   ", Ui_TSpeed_Change);
    oled_show_string(0, 3, txt);
    while (1)   //主循环
    {
        if (CFlag == 0)
        {
            if (KEY_Read(KEY2) == 0)      //按下KEY2键，占空比赠大
            {
                //gpio_set_level(P20_3, 1);
                if (Flag < 0)
                    Flag = 0;
                else
                    Flag = Flag - 1;
                system_delay_ms(100);
                //gpio_set_level(P20_3, 0);
            }
            if (KEY_Read(KEY3) == 0)      //按下KEY1键，选中参数进行调整
            {
                gpio_set_level(P20_3, 1);
                CFlag = 1;
                //LED_Ctrl(LED0, ON);
                system_delay_ms(100);
                gpio_set_level(P20_3, 0);
            }
            if (KEY_Read(KEY1) == 0)      //按下KEY0键，占空比减小
            {
                if (Flag > 3)
                    Flag = 3;
                else
                    Flag = Flag + 1;
                system_delay_ms(100);
            }
        }
        if (flag != Flag || cflag != CFlag)
        {
            flag = Flag;
            switch (Flag)
            {
            case 0:
            {
                sprintf(txt, "P: %05f; <<", Ui_RMotor_P);
                oled_show_string(0, 0, txt);
                sprintf(txt, "I: %05f;   ", Ui_RMotor_I);
                oled_show_string(0, 1, txt);
                sprintf(txt, "D: %05f;   ", Ui_RMotor_D);
                oled_show_string(0, 2, txt);
                sprintf(txt, "T: %05d;   ", Ui_TSpeed_Change);
                oled_show_string(0, 3, txt);
                if (CFlag == 1)
                {
                    if (KEY_Read(KEY2) == 0)      //按下KEY2键，占空比增大
                    {
                        Ui_RMotor_P = Ui_RMotor_P + 1;
                        RSpeed_pid.kp = RSpeed_pid.kp + 1;
                        system_delay_ms(100);
                    }
                    if (KEY_Read(KEY4) == 0)
                    {
                        gpio_set_level(P20_3, 1);
                        CFlag = 0;
                        system_delay_ms(100);
                        gpio_set_level(P20_3, 0);
                    }
                    if (KEY_Read(KEY1) == 0)      //按下KEY0键，占空比减小
                    {
                        Ui_RMotor_P = Ui_RMotor_P - 1;
                        RSpeed_pid.kp = RSpeed_pid.kp - 1;
                        system_delay_ms(100);
                    }
                }
                break;
            }
            case 1:
            {
                sprintf(txt, "P: %05f;   ", Ui_RMotor_P);
                oled_show_string(0, 0, txt);
                sprintf(txt, "I: %05f; <<", Ui_RMotor_I);
                oled_show_string(0, 1, txt);
                sprintf(txt, "D: %05f;   ", Ui_RMotor_D);
                oled_show_string(0, 2, txt);
                sprintf(txt, "T: %05d;   ", Ui_TSpeed_Change);
                oled_show_string(0, 3, txt);
                if (CFlag == 1)
                {
                    if (KEY_Read(KEY2) == 0)
                    {
                        Ui_RMotor_I = Ui_RMotor_I + 0.1;
                        RSpeed_pid.ki = RSpeed_pid.ki + 0.1;
                        system_delay_ms(100);
                    }
                    if (KEY_Read(KEY4) == 0)
                    {
                        gpio_set_level(P20_3, 1);
                        CFlag = 0;
                        system_delay_ms(100);
                        gpio_set_level(P20_3, 0);
                    }
                    if (KEY_Read(KEY1) == 0)
                    {
                        Ui_RMotor_I = Ui_RMotor_I - 0.1;
                        RSpeed_pid.ki = RSpeed_pid.ki - 0.1;
                        system_delay_ms(100);
                    }
                }
                break;
            }
            case 2:
            {
                sprintf(txt, "P: %05f;   ", Ui_RMotor_P);
                oled_show_string(0, 0, txt);
                sprintf(txt, "I: %05f;   ", Ui_RMotor_I);
                oled_show_string(0, 1, txt);
                sprintf(txt, "D: %05f;<< ", Ui_RMotor_D);
                oled_show_string(0, 2, txt);
                sprintf(txt, "T: %05d;   ", Ui_TSpeed_Change);
                oled_show_string(0, 3, txt);
                if (CFlag == 1)
                {
                    if (KEY_Read(KEY2) == 0)
                    {
                        Ui_RMotor_D = Ui_RMotor_D + 1;
                        RSpeed_pid.kd = RSpeed_pid.kd + 1;
                        system_delay_ms(100);
                    }
                    if (KEY_Read(KEY4) == 0)
                    {
                        gpio_set_level(P20_3, 1);
                        CFlag = 0;
                        system_delay_ms(100);
                        gpio_set_level(P20_3, 0);
                    }
                    if (KEY_Read(KEY1) == 0)
                    {
                        Ui_RMotor_D = Ui_RMotor_D - 1;
                        RSpeed_pid.kd = RSpeed_pid.kd - 1;
                        system_delay_ms(100);
                    }
                }
                break;
            }
            case 3:
            {
                sprintf(txt, "P: %05f;   ", Ui_RMotor_P);
                oled_show_string(0, 0, txt);
                sprintf(txt, "I: %05f;   ", Ui_RMotor_I);
                oled_show_string(0, 1, txt);
                sprintf(txt, "D: %05f;   ", Ui_RMotor_D);
                oled_show_string(0, 2, txt);
                sprintf(txt, "T: %05d; <<", Ui_TSpeed_Change);
                oled_show_string(0, 3, txt);
                if (CFlag == 1)
                {
                    if (KEY_Read(KEY2) == 0)
                    {
                        Ui_TSpeed_Change = Ui_TSpeed_Change + 1;
                        system_delay_ms(100);
                    }
                    if (KEY_Read(KEY4) == 0)
                    {
                        gpio_set_level(P20_3, 1);
                        CFlag = 0;
                        system_delay_ms(100);
                        gpio_set_level(P20_3, 0);
                    }
                    if (KEY_Read(KEY1) == 0)
                    {
                        Ui_TSpeed_Change = Ui_TSpeed_Change - 1;
                        system_delay_ms(100);
                    }
                }
                break;
            }
            default:;
            }
        }
        //参数复位s1,s4
        if ((KEY_Read(KEY5) == 0))
        {
            sprintf(txt, "P: %05f;   ", Ui_RMotor_P);
            oled_show_string(0, 0, txt);
            sprintf(txt, "I: %05f;   ", Ui_RMotor_I);
            oled_show_string(0, 1, txt);
            sprintf(txt, "D: %05f;<< ", Ui_RMotor_D);
            oled_show_string(0, 2, txt);
            sprintf(txt, "T: %05d;   ", Ui_TSpeed_Change);
            oled_show_string(0, 3, txt);
            CFlag = 0;
            system_delay_ms(100);
        }
        if ((KEY_Read(KEY6) == 0))
        {
            oled_clear();
            Ui_System();
            system_delay_ms(100);
        }
    }
}
void L_MotorUiSystem(void)
{
    int Flag = 0, flag = 0, CFlag = 0, cflag = 0;//定义调参ui的标志数，flag表示选择层数，cflag表示是否选中并进行调整
    char txt[16];
    sprintf(txt, "P: %05f;   ", Ui_LMotor_P);
    oled_show_string(0, 0, txt);
    sprintf(txt, "I: %05f;   ", Ui_LMotor_I);
    oled_show_string(0, 1, txt);
    sprintf(txt, "D: %05f;   ", Ui_LMotor_D);
    oled_show_string(0, 2, txt);
    sprintf(txt, "T: %05d;   ", Ui_TSpeed_Change);
    oled_show_string(0, 3, txt);
    while (1)   //主循环
    {
        if (CFlag == 0)
        {
            if (KEY_Read(KEY2) == 0)      //按下KEY2键，占空比赠大
            {
                //gpio_set_level(P20_3, 1);
                if (Flag < 0)
                    Flag = 0;
                else
                    Flag = Flag - 1;
                system_delay_ms(100);
                //gpio_set_level(P20_3, 0);
            }
            if (KEY_Read(KEY3) == 0)      //按下KEY1键，选中参数进行调整
            {
                gpio_set_level(P20_3, 1);
                CFlag = 1;
                //LED_Ctrl(LED0, ON);
                system_delay_ms(100);
                gpio_set_level(P20_3, 0);
            }
            if (KEY_Read(KEY1) == 0)      //按下KEY0键，占空比减小
            {
                if (Flag > 3)
                    Flag = 3;
                else
                    Flag = Flag + 1;
                system_delay_ms(100);
            }
        }
        if (flag != Flag || cflag != CFlag)
        {
            flag = Flag;
            switch (Flag)
            {
            case 0:
            {
                sprintf(txt, "P: %05f; <<", Ui_LMotor_P);
                oled_show_string(0, 0, txt);
                sprintf(txt, "I: %05f;   ", Ui_LMotor_I);
                oled_show_string(0, 1, txt);
                sprintf(txt, "D: %05f;   ", Ui_LMotor_D);
                oled_show_string(0, 2, txt);
                sprintf(txt, "T: %05d;   ", Ui_TSpeed_Change);
                oled_show_string(0, 3, txt);
                if (CFlag == 1)
                {
                    if (KEY_Read(KEY2) == 0)      //按下KEY2键，占空比增大
                    {
                        Ui_LMotor_P = Ui_LMotor_P + 1;
                        LSpeed_pid.kp = LSpeed_pid.kp + 1;
                        system_delay_ms(100);
                    }
                    if (KEY_Read(KEY4) == 0)
                    {
                        gpio_set_level(P20_3, 1);
                        CFlag = 0;
                        system_delay_ms(100);
                        gpio_set_level(P20_3, 0);
                    }
                    if (KEY_Read(KEY1) == 0)      //按下KEY0键，占空比减小
                    {
                        Ui_LMotor_P = Ui_LMotor_P - 1;
                        LSpeed_pid.kp = LSpeed_pid.kp - 1;
                        system_delay_ms(100);
                    }
                }
                break;
            }
            case 1:
            {
                sprintf(txt, "P: %05f;   ", Ui_LMotor_P);
                oled_show_string(0, 0, txt);
                sprintf(txt, "I: %05f; <<", Ui_LMotor_I);
                oled_show_string(0, 1, txt);
                sprintf(txt, "D: %05f;   ", Ui_LMotor_D);
                oled_show_string(0, 2, txt);
                sprintf(txt, "T: %05d;   ", Ui_TSpeed_Change);
                oled_show_string(0, 3, txt);
                if (CFlag == 1)
                {
                    if (KEY_Read(KEY2) == 0)
                    {
                        Ui_LMotor_I = Ui_LMotor_I + 0.1;
                        LSpeed_pid.ki = LSpeed_pid.ki + 0.1;
                        system_delay_ms(100);
                    }
                    if (KEY_Read(KEY4) == 0)
                    {
                        gpio_set_level(P20_3, 1);
                        CFlag = 0;
                        system_delay_ms(100);
                        gpio_set_level(P20_3, 0);
                    }
                    if (KEY_Read(KEY1) == 0)
                    {
                        Ui_LMotor_I = Ui_LMotor_I - 0.1;
                        LSpeed_pid.ki = LSpeed_pid.ki - 0.1;
                        system_delay_ms(100);
                    }
                }
                break;
            }
            case 2:
            {
                sprintf(txt, "P: %05f;   ", Ui_LMotor_P);
                oled_show_string(0, 0, txt);
                sprintf(txt, "I: %05f;   ", Ui_LMotor_I);
                oled_show_string(0, 1, txt);
                sprintf(txt, "D: %05f;<<", Ui_LMotor_D);
                oled_show_string(0, 2, txt);
                sprintf(txt, "T: %05d;   ", Ui_TSpeed_Change);
                oled_show_string(0, 3, txt);
                if (CFlag == 1)
                {
                    if (KEY_Read(KEY2) == 0)
                    {
                        Ui_LMotor_D = Ui_LMotor_D + 1;
                        LSpeed_pid.kd = LSpeed_pid.kd + 1;
                        system_delay_ms(100);
                    }
                    if (KEY_Read(KEY4) == 0)
                    {
                        gpio_set_level(P20_3, 1);
                        CFlag = 0;
                        system_delay_ms(100);
                        gpio_set_level(P20_3, 0);
                    }
                    if (KEY_Read(KEY1) == 0)
                    {
                        Ui_LMotor_D = Ui_LMotor_D - 1;
                        LSpeed_pid.kd = LSpeed_pid.kd - 1;
                        system_delay_ms(100);
                    }
                }
                break;
            }
            case 3:
            {
                sprintf(txt, "P: %05f;   ", Ui_LMotor_P);
                oled_show_string(0, 0, txt);
                sprintf(txt, "I: %05f;   ", Ui_LMotor_I);
                oled_show_string(0, 1, txt);
                sprintf(txt, "D: %05f;   ", Ui_LMotor_D);
                oled_show_string(0, 2, txt);
                sprintf(txt, "T: %05d; <<", Ui_TSpeed_Change);
                oled_show_string(0, 3, txt);
                if (CFlag == 1)
                {
                    if (KEY_Read(KEY2) == 0)
                    {
                        Ui_TSpeed_Change = Ui_TSpeed_Change + 1;
                        system_delay_ms(100);
                    }
                    if (KEY_Read(KEY4) == 0)
                    {
                        gpio_set_level(P20_3, 1);
                        CFlag = 0;
                        system_delay_ms(100);
                        gpio_set_level(P20_3, 0);
                    }
                    if (KEY_Read(KEY1) == 0)
                    {
                        Ui_TSpeed_Change = Ui_TSpeed_Change - 1;
                        system_delay_ms(100);
                    }
                }
                break;
            }
            default:;
            }
        }
        //参数复位s1,s4
        if ((KEY_Read(KEY5) == 0))
        {
            sprintf(txt, "P: %05f;   ", Ui_LMotor_P);
            oled_show_string(0, 0, txt);
            sprintf(txt, "I: %05f;   ", Ui_LMotor_I);
            oled_show_string(0, 1, txt);
            sprintf(txt, "D: %05f;   ", Ui_LMotor_D);
            oled_show_string(0, 2, txt);
            sprintf(txt, "T: %05d;   ", Ui_TSpeed_Change);
            oled_show_string(0, 3, txt);
            CFlag = 0;
            system_delay_ms(100);
        }
        if ((KEY_Read(KEY6) == 0))
        {
            oled_clear();
            Ui_System();
            system_delay_ms(100);
        }
    }
}
void SteerUiSystem(void)
{
    int Flag = 0, flag = 0, CFlag = 0, cflag = 0;//定义调参ui的标志数，flag表示选择层数，cflag表示是否选中并进行调整
    char txt[16];
    sprintf(txt, "P: %05f;   ", Ui_Steer_P);
    oled_show_string(0, 0, txt);
    sprintf(txt, "D: %05f;   ", Ui_Steer_D);
    oled_show_string(0, 1, txt);
    sprintf(txt, "Line: %d;   ", Ui_Control_Line);
    oled_show_string(0, 2, txt);
    sprintf(txt, "Stra: %05f;   ", Ui_Straight_Diff_K);
    oled_show_string(0, 3, txt);
    sprintf(txt, "Bend: %05f;   ", Ui_Bend_Diff_K);
    oled_show_string(0, 4, txt);

    while (1)   //主循环
    {
        if (CFlag == 0)//选择调参内容  2为向上 0为向下 1为确认 可以循环使用
        {
            if (KEY_Read(KEY2) == 0)      //按下KEY2键，占空比赠大
            {
                //gpio_set_level(P20_3, 1);
                if (Flag < 0)
                    Flag = 0;
                else
                    Flag = Flag - 1;
                system_delay_ms(100);
                //gpio_set_level(P20_3, 0);
            }
            if (KEY_Read(KEY3) == 0)      //按下KEY1键，选中参数进行调整
            {
                gpio_set_level(P20_3, 1);
                gpio_set_level(P20_8, 0);
                CFlag = 1;
                system_delay_ms(100);
                gpio_set_level(P20_3, 0);
            }
            if (KEY_Read(KEY1) == 0)      //按下KEY0键，占空比减小
            {
                if (Flag > 4)
                    Flag = 4;
                else
                    Flag = Flag + 1;
                system_delay_ms(100);
            }
        }
        if (flag != Flag || cflag != CFlag)
        {
            flag = Flag;
            switch (Flag)
            {
            case 0:
            {
                sprintf(txt, "P: %05f; <<", Ui_Steer_P);
                oled_show_string(0, 0, txt);
                sprintf(txt, "D: %05f;   ", Ui_Steer_D);
                oled_show_string(0, 1, txt);
                sprintf(txt, "Line: %d;   ", Ui_Control_Line);
                oled_show_string(0, 2, txt);
                sprintf(txt, "Stra: %05f;   ", Ui_Straight_Diff_K);
                oled_show_string(0, 3, txt);
                sprintf(txt, "Bend: %05f;   ", Ui_Bend_Diff_K);
                oled_show_string(0, 4, txt);
                if (CFlag == 1)
                {
                    if (KEY_Read(KEY2) == 0)      //按下KEY2键，占空比增大
                    {
                        Ui_Steer_P = Ui_Steer_P + 0.01;
                        Steer_pid.kp = Steer_pid.kp + 0.01;
                        system_delay_ms(100);
                    }
                    if (KEY_Read(KEY4) == 0)
                    {
                        gpio_set_level(P20_3, 1);
                        gpio_set_level(P20_8, 1);
                        CFlag = 0;
                        system_delay_ms(100);
                        gpio_set_level(P20_3, 0);
                    }
                    if (KEY_Read(KEY1) == 0)      //按下KEY0键，占空比减小
                    {
                        Ui_Steer_P = Ui_Steer_P - 0.01;
                        Steer_pid.kp = Steer_pid.kp - 0.01;
                        system_delay_ms(100);
                    }
                }
                break;
            }
            case 1:
            {
                sprintf(txt, "P: %05f;   ", Ui_Steer_P);
                oled_show_string(0, 0, txt);
                sprintf(txt, "D: %05f; <<", Ui_Steer_D);
                oled_show_string(0, 1, txt);
                sprintf(txt, "Line: %d;   ", Ui_Control_Line);
                oled_show_string(0, 2, txt);
                sprintf(txt, "Stra: %05f;   ", Ui_Straight_Diff_K);
                oled_show_string(0, 3, txt);
                sprintf(txt, "Bend: %05f;   ", Ui_Bend_Diff_K);
                oled_show_string(0, 4, txt);
                if (CFlag == 1)
                {
                    if (KEY_Read(KEY2) == 0)
                    {
                        Ui_Steer_D = Ui_Steer_D + 0.1;
                        Steer_pid.kd = Steer_pid.kd + 0.1;
                        system_delay_ms(100);
                    }
                    if (KEY_Read(KEY4) == 0)
                    {
                        gpio_set_level(P20_3, 1);
                        gpio_set_level(P20_8, 1);
                        CFlag = 0;
                        system_delay_ms(500);
                        gpio_set_level(P20_3, 0);
                    }
                    if (KEY_Read(KEY1) == 0)
                    {
                        Ui_Steer_D = Ui_Steer_D - 0.1;
                        Steer_pid.kd = Steer_pid.kd - 0.1;
                        system_delay_ms(100);
                    }
                }
                break;
            }
            case 2:
            {
                sprintf(txt, "P: %05f;   ", Ui_Steer_P);
                oled_show_string(0, 0, txt);
                sprintf(txt, "D: %05f;   ", Ui_Steer_D);
                oled_show_string(0, 1, txt);
                sprintf(txt, "Line: %d; <<", Ui_Control_Line);
                oled_show_string(0, 2, txt);
                sprintf(txt, "Stra: %05f;   ", Ui_Straight_Diff_K);
                oled_show_string(0, 3, txt);
                sprintf(txt, "Bend: %05f;   ", Ui_Bend_Diff_K);
                oled_show_string(0, 4, txt);
                if (CFlag == 1)
                {
                    if (KEY_Read(KEY2) == 0)
                    {
                        Ui_Control_Line = Ui_Control_Line + 1;
                        system_delay_ms(100);
                    }
                    if (KEY_Read(KEY4) == 0)
                    {
                        gpio_set_level(P20_3, 1);
                        gpio_set_level(P20_8, 1);
                        CFlag = 0;
                        system_delay_ms(100);
                        gpio_set_level(P20_3, 0);
                    }
                    if (KEY_Read(KEY1) == 0)
                    {
                        Ui_Control_Line = Ui_Control_Line - 1;
                        system_delay_ms(100);
                    }
                }
                break;
            }
            case 3:
            {
                sprintf(txt, "P: %05f;   ", Ui_Steer_P);
                oled_show_string(0, 0, txt);
                sprintf(txt, "D: %05f;   ", Ui_Steer_D);
                oled_show_string(0, 1, txt);
                sprintf(txt, "Line: %d;   ", Ui_Control_Line);
                oled_show_string(0, 2, txt);
                sprintf(txt, "Stra: %05f; <<", Ui_Straight_Diff_K);
                oled_show_string(0, 3, txt);
                sprintf(txt, "Bend: %05f;   ", Ui_Bend_Diff_K);
                oled_show_string(0, 4, txt);
                if (CFlag == 1)
                {
                    if (KEY_Read(KEY2) == 0)
                    {
                        Ui_Straight_Diff_K = Ui_Straight_Diff_K + 0.001;
                        system_delay_ms(100);
                    }
                    if (KEY_Read(KEY4) == 0)
                    {
                        gpio_set_level(P20_3, 1);
                        gpio_set_level(P20_8, 1);
                        CFlag = 0;
                        system_delay_ms(100);
                        gpio_set_level(P20_3, 0);
                    }
                    if (KEY_Read(KEY1) == 0)
                    {
                        Ui_Straight_Diff_K = Ui_Straight_Diff_K - 0.001;
                        system_delay_ms(100);
                    }
                }
                break;
            }
            case 4:
            {
                sprintf(txt, "P: %05f;   ", Ui_Steer_P);
                oled_show_string(0, 0, txt);
                sprintf(txt, "D: %05f;   ", Ui_Steer_D);
                oled_show_string(0, 1, txt);
                sprintf(txt, "Line: %d;   ", Ui_Control_Line);
                oled_show_string(0, 2, txt);
                sprintf(txt, "Stra: %05f;   ", Ui_Straight_Diff_K);
                oled_show_string(0, 3, txt);
                sprintf(txt, "Bend: %05f; <<", Ui_Bend_Diff_K);
                oled_show_string(0, 4, txt);
                if (CFlag == 1)
                {
                    if (KEY_Read(KEY2) == 0)
                    {
                        Ui_Bend_Diff_K = Ui_Bend_Diff_K + 0.001;
                        system_delay_ms(100);
                    }
                    if (KEY_Read(KEY4) == 0)
                    {
                        gpio_set_level(P20_3, 1);
                        gpio_set_level(P20_8, 1);
                        CFlag = 0;
                        system_delay_ms(100);
                        gpio_set_level(P20_3, 0);
                    }
                    if (KEY_Read(KEY1) == 0)
                    {
                        Ui_Bend_Diff_K = Ui_Bend_Diff_K - 0.001;
                        system_delay_ms(100);
                    }
                }
                break;
            }

            default:;
            }
        }

        //参数复位s1,s4
        if ((KEY_Read(KEY5) == 0))
        {
            sprintf(txt, "P: %05f;   ", Ui_Steer_P);
            oled_show_string(0, 0, txt);
            sprintf(txt, "D: %05f;   ", Ui_Steer_D);
            oled_show_string(0, 1, txt);
            sprintf(txt, "Line: %d;   ", Ui_Control_Line);
            oled_show_string(0, 2, txt);
            sprintf(txt, "Stra: %05f;   ", Ui_Straight_Diff_K);
            oled_show_string(0, 3, txt);
            sprintf(txt, "Bend: %05f;   ", Ui_Bend_Diff_K);
            oled_show_string(0, 4, txt);
            Flag_Stop = 0;
            CFlag = 0;
            system_delay_ms(100);
        }
        if ((KEY_Read(KEY6) == 0))
        {
            oled_clear();
            Ui_System();
            system_delay_ms(100);
        }
    }
}
//void SteerUiSystem(void)
//{
//    int Flag = 0, flag = 0, CFlag = 0, cflag = 0;//定义调参ui的标志数，flag表示选择层数，cflag表示是否选中并进行调整
//    char txt[16];
//    sprintf(txt, "P: %05f;   ", Ui_Steer_P);
//    oled_show_string(0, 0, txt);
//    sprintf(txt, "D: %05f;   ", Ui_Steer_D);
//    oled_show_string(0, 1, txt);
//    sprintf(txt, "1: %d;   ", Ui_Servo_Mid);
//    oled_show_string(0, 2, txt);
//    sprintf(txt, "Stra: %05f;   ", Ui_Straight_Diff_K);
//    oled_show_string(0, 3, txt);
//    sprintf(txt, "Bend: %05f;   ", Ui_Bend_Diff_K);
//    oled_show_string(0, 4, txt);
//
//    while (1)   //主循环
//    {
//        if (CFlag == 0)
//        {
//            if (KEY_Read(KEY2) == 0)      //按下KEY2键，占空比赠大
//            {
//                //gpio_set_level(P20_3, 1);
//                if (Flag < 0)
//                    Flag = 0;
//                else
//                    Flag = Flag - 1;
//                system_delay_ms(100);
//                //gpio_set_level(P20_3, 0);
//            }
//            if (KEY_Read(KEY3) == 0)      //按下KEY1键，选中参数进行调整
//            {
//                gpio_set_level(P20_3, 1);
//                gpio_set_level(P20_8, 0);
//                CFlag = 1;
//                system_delay_ms(100);
//                gpio_set_level(P20_3, 0);
//            }
//            if (KEY_Read(KEY1) == 0)      //按下KEY0键，占空比减小
//            {
//                if (Flag > 4)
//                    Flag = 4;
//                else
//                    Flag = Flag + 1;
//                system_delay_ms(100);
//            }
//        }
//        if (flag != Flag || cflag != CFlag)
//        {
//            flag = Flag;
//            switch (Flag)
//            {
//            case 0:
//            {
//                sprintf(txt, "P: %05f; <<", Ui_Steer_P);
//                oled_show_string(0, 0, txt);
//                sprintf(txt, "D: %05f;   ", Ui_Steer_D);
//                oled_show_string(0, 1, txt);
//                sprintf(txt, "1: %d;   ", Ui_Servo_Mid);
//                oled_show_string(0, 2, txt);
//                sprintf(txt, "Stra: %05f;   ", Ui_Straight_Diff_K);
//                oled_show_string(0, 3, txt);
//                sprintf(txt, "Bend: %05f;   ", Ui_Bend_Diff_K);
//                oled_show_string(0, 4, txt);
//                if (CFlag == 1)
//                {
//                    if (KEY_Read(KEY2) == 0)      //按下KEY2键，占空比增大
//                    {
//                        Ui_Steer_P = Ui_Steer_P + 0.01;
//                        Steer_pid.kp = Steer_pid.kp + 0.01;
//                        system_delay_ms(100);
//                    }
//                    if (KEY_Read(KEY6) == 0)
//                    {
//                        gpio_set_level(P20_3, 1);
//                        gpio_set_level(P20_8, 1);
//                        CFlag = 0;
//                        system_delay_ms(100);
//                        gpio_set_level(P20_3, 0);
//                    }
//                    if (KEY_Read(KEY1) == 0)      //按下KEY0键，占空比减小
//                    {
//                        Ui_Steer_P = Ui_Steer_P - 0.01;
//                        Steer_pid.kp = Steer_pid.kp - 0.01;
//                        system_delay_ms(100);
//                    }
//                }
//                break;
//            }
//            case 1:
//            {
//                sprintf(txt, "P: %05f;   ", Ui_Steer_P);
//                oled_show_string(0, 0, txt);
//                sprintf(txt, "D: %05f; <<", Ui_Steer_D);
//                oled_show_string(0, 1, txt);
//                sprintf(txt, "1: %d;   ", Ui_Servo_Mid);
//                oled_show_string(0, 2, txt);
//                sprintf(txt, "Stra: %05f;   ", Ui_Straight_Diff_K);
//                oled_show_string(0, 3, txt);
//                sprintf(txt, "Bend: %05f;   ", Ui_Bend_Diff_K);
//                oled_show_string(0, 4, txt);
//                if (CFlag == 1)
//                {
//                    if (KEY_Read(KEY2) == 0)
//                    {
//                        Ui_Steer_D = Ui_Steer_D + 0.1;
//                        Steer_pid.kd = Steer_pid.kd + 0.1;
//                        system_delay_ms(100);
//                    }
//                    if (KEY_Read(KEY6) == 0)
//                    {
//                        gpio_set_level(P20_3, 1);
//                        gpio_set_level(P20_8, 1);
//                        CFlag = 0;
//                        system_delay_ms(500);
//                        gpio_set_level(P20_3, 0);
//                    }
//                    if (KEY_Read(KEY1) == 0)
//                    {
//                        Ui_Steer_D = Ui_Steer_D - 0.1;
//                        Steer_pid.kd = Steer_pid.kd - 0.1;
//                        system_delay_ms(100);
//                    }
//                }
//                break;
//            }
//            case 2:
//            {
//                sprintf(txt, "P: %05f;   ", Ui_Steer_P);
//                oled_show_string(0, 0, txt);
//                sprintf(txt, "D: %05f;   ", Ui_Steer_D);
//                oled_show_string(0, 1, txt);
//                sprintf(txt, "1: %d; <<", Ui_Servo_Mid);
//                oled_show_string(0, 2, txt);
//                sprintf(txt, "Stra: %05f;   ", Ui_Straight_Diff_K);
//                oled_show_string(0, 3, txt);
//                sprintf(txt, "Bend: %05f;   ", Ui_Bend_Diff_K);
//                oled_show_string(0, 4, txt);
//                if (CFlag == 1)
//                {
//                    if (KEY_Read(KEY2) == 0)
//                    {
//                        Ui_Servo_Mid = Ui_Servo_Mid + 1;
//                        system_delay_ms(100);
//                    }
//                    if (KEY_Read(KEY6) == 0)
//                    {
//                        gpio_set_level(P20_3, 1);
//                        gpio_set_level(P20_8, 1);
//                        CFlag = 0;
//                        system_delay_ms(100);
//                        gpio_set_level(P20_3, 0);
//                    }
//                    if (KEY_Read(KEY1) == 0)
//                    {
//                        Ui_Servo_Mid = Ui_Servo_Mid - 1;
//                        system_delay_ms(100);
//                    }
//                }
//                break;
//            }
//            case 3:
//            {
//                sprintf(txt, "P: %05f;   ", Ui_Steer_P);
//                oled_show_string(0, 0, txt);
//                sprintf(txt, "D: %05f;   ", Ui_Steer_D);
//                oled_show_string(0, 1, txt);
//                sprintf(txt, "1: %d;   ", Ui_Servo_Mid);
//                oled_show_string(0, 2, txt);
//                sprintf(txt, "Stra: %05f; <<", Ui_Straight_Diff_K);
//                oled_show_string(0, 3, txt);
//                sprintf(txt, "Bend: %05f;   ", Ui_Bend_Diff_K);
//                oled_show_string(0, 4, txt);
//                if (CFlag == 1)
//                {
//                    if (KEY_Read(KEY2) == 0)
//                    {
//                        Ui_Straight_Diff_K = Ui_Straight_Diff_K + 0.001;
//                        system_delay_ms(100);
//                    }
//                    if (KEY_Read(KEY6) == 0)
//                    {
//                        gpio_set_level(P20_3, 1);
//                        gpio_set_level(P20_8, 1);
//                        CFlag = 0;
//                        system_delay_ms(100);
//                        gpio_set_level(P20_3, 0);
//                    }
//                    if (KEY_Read(KEY1) == 0)
//                    {
//                        Ui_Straight_Diff_K = Ui_Straight_Diff_K - 0.001;
//                        system_delay_ms(100);
//                    }
//                }
//                break;
//            }
//            case 4:
//            {
//                sprintf(txt, "P: %05f;   ", Ui_Steer_P);
//                oled_show_string(0, 0, txt);
//                sprintf(txt, "D: %05f;   ", Ui_Steer_D);
//                oled_show_string(0, 1, txt);
//                sprintf(txt, "1: %d;   ", Ui_Servo_Mid);
//                oled_show_string(0, 2, txt);
//                sprintf(txt, "Stra: %05f;   ", Ui_Straight_Diff_K);
//                oled_show_string(0, 3, txt);
//                sprintf(txt, "Bend: %05f; <<", Ui_Bend_Diff_K);
//                oled_show_string(0, 4, txt);
//                if (CFlag == 1)
//                {
//                    if (KEY_Read(KEY2) == 0)
//                    {
//                        Ui_Bend_Diff_K = Ui_Bend_Diff_K + 0.001;
//                        system_delay_ms(100);
//                    }
//                    if (KEY_Read(KEY6) == 0)
//                    {
//                        gpio_set_level(P20_3, 1);
//                        gpio_set_level(P20_8, 1);
//                        CFlag = 0;
//                        system_delay_ms(100);
//                        gpio_set_level(P20_3, 0);
//                    }
//                    if (KEY_Read(KEY1) == 0)
//                    {
//                        Ui_Bend_Diff_K = Ui_Bend_Diff_K - 0.001;
//                        system_delay_ms(100);
//                    }
//                }
//                break;
//            }
//
//            default:;
//            }
//        }
//
//        //参数复位s1,s4
//        if ((KEY_Read(KEY5) == 0))
//        {
//            sprintf(txt, "P: %05f;   ", Ui_Steer_P);
//            oled_show_string(0, 0, txt);
//            sprintf(txt, "D: %05f;   ", Ui_Steer_D);
//            oled_show_string(0, 1, txt);
//            sprintf(txt, "1: %d;   ", Ui_Servo_Mid);
//            oled_show_string(0, 2, txt);
//            sprintf(txt, "Stra: %05f;   ", Ui_Straight_Diff_K);
//            oled_show_string(0, 3, txt);
//            sprintf(txt, "Bend: %05f;   ", Ui_Bend_Diff_K);
//            oled_show_string(0, 4, txt);
//            Flag_Stop = 0;
//            CFlag = 0;
//            system_delay_ms(100);
//        }
//    }
//}
void Fork_UiSystem(void)
{
    int Flag = 0, flag = 0, CFlag = 0, cflag = 0;//定义调参ui的标志数，flag表示选择层数，cflag表示是否选中并进行调整
    char txt[16];
    sprintf(txt, "Fork_Speed: %d   ;   ", Ui_Fork_Speed);
    oled_show_string(0, 0, txt);
    sprintf(txt, "Road_Speed: %d   ;   ", Ui_ForkRoad_Speed);
    oled_show_string(0, 1, txt);

    while (1)   //主循环
    {
        if (CFlag == 0)//选择调参内容  2为向上 0为向下 1为确认 可以循环使用
        {
            if (KEY_Read(KEY2) == 0)      //按下KEY2键，占空比赠大
            {
                //gpio_set_level(P20_3, 1);
                if (Flag < 0)
                    Flag = 0;
                else
                    Flag = Flag - 1;
                system_delay_ms(100);
                //gpio_set_level(P20_3, 0);
            }
            if (KEY_Read(KEY3) == 0)      //按下KEY1键，选中参数进行调整
            {
                gpio_set_level(P20_3, 1);
                gpio_set_level(P20_8, 0);
                CFlag = 1;
                system_delay_ms(100);
                gpio_set_level(P20_3, 0);
            }
            if (KEY_Read(KEY1) == 0)      //按下KEY0键，占空比减小
            {
                if (Flag > 1)
                    Flag = 1;
                else
                    Flag = Flag + 1;
                system_delay_ms(100);
            }
        }
        if (flag != Flag || cflag != CFlag)
        {
            flag = Flag;
            switch (Flag)
            {
            case 0:
            {
                sprintf(txt, "Fork_Speed: %d <<;   ", Ui_Fork_Speed);
                oled_show_string(0, 0, txt);
                sprintf(txt, "Road_Speed: %d   ;   ", Ui_ForkRoad_Speed);
                oled_show_string(0, 1, txt);
                if (CFlag == 1)
                {
                    if (KEY_Read(KEY2) == 0)      //按下KEY2键，占空比增大
                    {
                        Ui_Fork_Speed = Ui_Fork_Speed + 1;
                        system_delay_ms(100);
                    }
                    if (KEY_Read(KEY4) == 0)
                    {
                        gpio_set_level(P20_3, 1);
                        gpio_set_level(P20_8, 1);
                        CFlag = 0;
                        system_delay_ms(100);
                        gpio_set_level(P20_3, 0);
                    }
                    if (KEY_Read(KEY1) == 0)      //按下KEY0键，占空比减小
                    {
                        Ui_Fork_Speed = Ui_Fork_Speed - 1;
                        system_delay_ms(100);
                    }
                }
                break;
            }
            case 1:
            {
                sprintf(txt, "Fork_Speed: %d   ;   ", Ui_Fork_Speed);
                oled_show_string(0, 0, txt);
                sprintf(txt, "Road_Speed: %d <<;   ", Ui_ForkRoad_Speed);
                oled_show_string(0, 1, txt);
                if (CFlag == 1)
                {
                    if (KEY_Read(KEY2) == 0)
                    {
                        Ui_ForkRoad_Speed = Ui_ForkRoad_Speed + 1;
                        system_delay_ms(100);
                    }
                    if (KEY_Read(KEY4) == 0)
                    {
                        gpio_set_level(P20_3, 1);
                        gpio_set_level(P20_8, 1);
                        CFlag = 0;
                        system_delay_ms(500);
                        gpio_set_level(P20_3, 0);
                    }
                    if (KEY_Read(KEY1) == 0)
                    {
                        Ui_ForkRoad_Speed = Ui_ForkRoad_Speed - 1;
                        system_delay_ms(100);
                    }
                }
                break;
            }
            default:
                break;
            }
        }
        if ((KEY_Read(KEY5) == 0))
        {
            sprintf(txt, "Fork_Speed: %d   ;   ", Ui_Fork_Speed);
            oled_show_string(0, 0, txt);
            sprintf(txt, "Road_Speed: %d   ;   ", Ui_ForkRoad_Speed);
            oled_show_string(0, 1, txt);
            Flag_Stop = 0;
            CFlag = 0;
            system_delay_ms(100);
        }
        if ((KEY_Read(KEY6) == 0))
        {
            oled_clear();
            Ui_System();
            system_delay_ms(100);
        }
    }
}
void Ring_UiSystem(void)
{
    int Flag = 0, flag = 0, CFlag = 0, cflag = 0;//定义调参ui的标志数，flag表示选择层数，cflag表示是否选中并进行调整
    char txt[16];
    sprintf(txt, "Ui_Ring_Speed: %d   ;   ", Ui_Ring_Speed);
    oled_show_string(0, 0, txt);

    while (1)   //主循环
    {
        if (CFlag == 0)//选择调参内容  2为向上 0为向下 1为确认 可以循环使用
        {
            //if (KEY_Read(KEY2) == 0)      //按下KEY2键，占空比赠大
            //{
            //    //gpio_set_level(P20_3, 1);
            //    if (Flag < 0)
            //        Flag = 0;
            //    else
            //        Flag = Flag - 1;
            //    system_delay_ms(100);
            //    //gpio_set_level(P20_3, 0);
            //}
            if (KEY_Read(KEY3) == 0)      //按下KEY1键，选中参数进行调整
            {
                gpio_set_level(P20_3, 1);
                gpio_set_level(P20_8, 0);
                CFlag = 1;
                system_delay_ms(100);
                gpio_set_level(P20_3, 0);
            }
            //if (KEY_Read(KEY1) == 0)      //按下KEY0键，占空比减小
            //{
            //    if (Flag > 4)
            //        Flag = 4;
            //    else
            //        Flag = Flag + 1;
            //    system_delay_ms(100);
            //}
        }
        if (flag != Flag || cflag != CFlag)
        {
            flag = Flag;
            switch (Flag)
            {
            case 0:
            {
                sprintf(txt, "Ui_Ring_Speed: %d <<;   ", Ui_Ring_Speed);
                oled_show_string(0, 0, txt);
                if (CFlag == 1)
                {
                    if (KEY_Read(KEY2) == 0)      //按下KEY2键，占空比增大
                    {
                        Ui_Ring_Speed = Ui_Ring_Speed + 1;
                        system_delay_ms(100);
                    }
                    if (KEY_Read(KEY4) == 0)
                    {
                        gpio_set_level(P20_3, 1);
                        gpio_set_level(P20_8, 1);
                        CFlag = 0;
                        system_delay_ms(100);
                        gpio_set_level(P20_3, 0);
                    }
                    if (KEY_Read(KEY1) == 0)      //按下KEY0键，占空比减小
                    {
                        Ui_Ring_Speed = Ui_Ring_Speed - 1;
                        system_delay_ms(100);
                    }
                }
                break;
            }
            default:
                break;
            }
        }
        if ((KEY_Read(KEY5) == 0))
        {
            sprintf(txt, "Ui_Ring_Speed: %d   ;   ", Ui_Ring_Speed);
            oled_show_string(0, 0, txt);
            Flag_Stop = 0;
            CFlag = 0;
            system_delay_ms(100);
        }
        if ((KEY_Read(KEY6) == 0))
        {
            oled_clear();
            Ui_System();
            system_delay_ms(100);
        }
    }
}
void T_UiSystem(void)
{

}
void Speed_UiSystem(void)
{
    int Flag = 0, flag = 0, CFlag = 0, cflag = 0;//定义调参ui的标志数，flag表示选择层数，cflag表示是否选中并进行调整
    char txt[16];
    sprintf(txt, "Ui_Max_Speed:   %d  ;   ", Ui_Max_Speed);
    oled_show_string(0, 0, txt);
    sprintf(txt, "Ui_Min_Speed:   %d  ;   ", Ui_Min_Speed);
    oled_show_string(0, 1, txt);
    sprintf(txt, "Ui_Lost_Speed:  %d  ;   ", Ui_Center_Lost_Speed);
    oled_show_string(0, 2, txt);
    sprintf(txt, "Ui_Inner_Speed: %d  ;   ", Ui_Inner_Speed);
    oled_show_string(0, 3, txt);
    sprintf(txt, "Ui_Stop_Speed:  %d  ;   ", Ui_Stop_Speed);
    oled_show_string(0, 4, txt);

    while (1)   //主循环
    {
        if (CFlag == 0)//选择调参内容  2为向上 0为向下 1为确认 可以循环使用
        {
            if (KEY_Read(KEY2) == 0)      //按下KEY2键，占空比赠大
            {
                //gpio_set_level(P20_3, 1);
                if (Flag < 0)
                    Flag = 0;
                else
                    Flag = Flag - 1;
                system_delay_ms(100);
                //gpio_set_level(P20_3, 0);
            }
            if (KEY_Read(KEY3) == 0)      //按下KEY1键，选中参数进行调整
            {
                gpio_set_level(P20_3, 1);
                gpio_set_level(P20_8, 0);
                CFlag = 1;
                system_delay_ms(100);
                gpio_set_level(P20_3, 0);
            }
            if (KEY_Read(KEY1) == 0)      //按下KEY0键，占空比减小
            {
                if (Flag > 4)
                    Flag = 4;
                else
                    Flag = Flag + 1;
                system_delay_ms(100);
            }
        }
        if (flag != Flag || cflag != CFlag)
        {
            flag = Flag;
            switch (Flag)
            {
            case 0:
            {
                sprintf(txt, "Ui_Max_Speed:   %d<<;   ", Ui_Max_Speed);
                oled_show_string(0, 0, txt);
                sprintf(txt, "Ui_Min_Speed:   %d  ;   ", Ui_Min_Speed);
                oled_show_string(0, 1, txt);
                sprintf(txt, "Ui_Lost_Speed:  %d  ;   ", Ui_Center_Lost_Speed);
                oled_show_string(0, 2, txt);
                sprintf(txt, "Ui_Inner_Speed: %d  ;   ", Ui_Inner_Speed);
                oled_show_string(0, 3, txt);
                sprintf(txt, "Ui_Stop_Speed:  %d  ;   ", Ui_Stop_Speed);
                oled_show_string(0, 4, txt);
                if (CFlag == 1)
                {
                    if (KEY_Read(KEY2) == 0)      //按下KEY2键，占空比增大
                    {
                        Ui_Max_Speed = Ui_Max_Speed + 1;
                        system_delay_ms(100);
                    }
                    if (KEY_Read(KEY4) == 0)
                    {
                        gpio_set_level(P20_3, 1);
                        gpio_set_level(P20_8, 1);
                        CFlag = 0;
                        system_delay_ms(100);
                        gpio_set_level(P20_3, 0);
                    }
                    if (KEY_Read(KEY1) == 0)      //按下KEY0键，占空比减小
                    {
                        Ui_Max_Speed = Ui_Max_Speed - 1;
                        system_delay_ms(100);
                    }
                }
                break;
            }
            case 1:
            {
                sprintf(txt, "Ui_Max_Speed:   %d  ;   ", Ui_Max_Speed);
                oled_show_string(0, 0, txt);
                sprintf(txt, "Ui_Min_Speed:   %d<<;   ", Ui_Min_Speed);
                oled_show_string(0, 1, txt);
                sprintf(txt, "Ui_Lost_Speed:  %d  ;   ", Ui_Center_Lost_Speed);
                oled_show_string(0, 2, txt);
                sprintf(txt, "Ui_Inner_Speed: %d  ;   ", Ui_Inner_Speed);
                oled_show_string(0, 3, txt);
                sprintf(txt, "Ui_Stop_Speed:  %d  ;   ", Ui_Stop_Speed);
                oled_show_string(0, 4, txt);
                if (CFlag == 1)
                {
                    if (KEY_Read(KEY2) == 0)      //按下KEY2键，占空比增大
                    {
                        Ui_Min_Speed = Ui_Min_Speed + 1;
                        system_delay_ms(100);
                    }
                    if (KEY_Read(KEY4) == 0)
                    {
                        gpio_set_level(P20_3, 1);
                        gpio_set_level(P20_8, 1);
                        CFlag = 0;
                        system_delay_ms(100);
                        gpio_set_level(P20_3, 0);
                    }
                    if (KEY_Read(KEY1) == 0)      //按下KEY0键，占空比减小
                    {
                        Ui_Min_Speed = Ui_Min_Speed - 1;
                        system_delay_ms(100);
                    }
                }
                break;
            }
            case 2:
            {
                sprintf(txt, "Ui_Max_Speed:   %d  ;   ", Ui_Max_Speed);
                oled_show_string(0, 0, txt);
                sprintf(txt, "Ui_Min_Speed:   %d  ;   ", Ui_Min_Speed);
                oled_show_string(0, 1, txt);
                sprintf(txt, "Ui_Lost_Speed:  %d<<;   ", Ui_Center_Lost_Speed);
                oled_show_string(0, 2, txt);
                sprintf(txt, "Ui_Inner_Speed: %d  ;   ", Ui_Inner_Speed);
                oled_show_string(0, 3, txt);
                sprintf(txt, "Ui_Stop_Speed:  %d  ;   ", Ui_Stop_Speed);
                oled_show_string(0, 4, txt);
                if (CFlag == 1)
                {
                    if (KEY_Read(KEY2) == 0)      //按下KEY2键，占空比增大
                    {
                        Ui_Center_Lost_Speed = Ui_Center_Lost_Speed + 1;
                        system_delay_ms(100);
                    }
                    if (KEY_Read(KEY4) == 0)
                    {
                        gpio_set_level(P20_3, 1);
                        gpio_set_level(P20_8, 1);
                        CFlag = 0;
                        system_delay_ms(100);
                        gpio_set_level(P20_3, 0);
                    }
                    if (KEY_Read(KEY1) == 0)      //按下KEY0键，占空比减小
                    {
                        Ui_Center_Lost_Speed = Ui_Center_Lost_Speed - 1;
                        system_delay_ms(100);
                    }
                }
                break;
            }
            case 3:
            {
                sprintf(txt, "Ui_Max_Speed:   %d  ;   ", Ui_Max_Speed);
                oled_show_string(0, 0, txt);
                sprintf(txt, "Ui_Min_Speed:   %d  ;   ", Ui_Min_Speed);
                oled_show_string(0, 1, txt);
                sprintf(txt, "Ui_Lost_Speed:  %d  ;   ", Ui_Center_Lost_Speed);
                oled_show_string(0, 2, txt);
                sprintf(txt, "Ui_Inner_Speed: %d<<;   ", Ui_Inner_Speed);
                oled_show_string(0, 3, txt);
                sprintf(txt, "Ui_Stop_Speed:  %d  ;   ", Ui_Stop_Speed);
                oled_show_string(0, 4, txt);
                if (CFlag == 1)
                {
                    if (KEY_Read(KEY2) == 0)      //按下KEY2键，占空比增大
                    {
                        Ui_Inner_Speed = Ui_Inner_Speed + 1;
                        system_delay_ms(100);
                    }
                    if (KEY_Read(KEY4) == 0)
                    {
                        gpio_set_level(P20_3, 1);
                        gpio_set_level(P20_8, 1);
                        CFlag = 0;
                        system_delay_ms(100);
                        gpio_set_level(P20_3, 0);
                    }
                    if (KEY_Read(KEY1) == 0)      //按下KEY0键，占空比减小
                    {
                        Ui_Inner_Speed = Ui_Inner_Speed - 1;
                        system_delay_ms(100);
                    }
                }
                break;
            }
            case 4:
            {
                sprintf(txt, "Ui_Max_Speed:   %d  ;   ", Ui_Max_Speed);
                oled_show_string(0, 0, txt);
                sprintf(txt, "Ui_Min_Speed:   %d  ;   ", Ui_Min_Speed);
                oled_show_string(0, 1, txt);
                sprintf(txt, "Ui_Lost_Speed:  %d  ;   ", Ui_Center_Lost_Speed);
                oled_show_string(0, 2, txt);
                sprintf(txt, "Ui_Inner_Speed: %d  ;   ", Ui_Inner_Speed);
                oled_show_string(0, 3, txt);
                sprintf(txt, "Ui_Stop_Speed:  %d<<;   ", Ui_Stop_Speed);
                oled_show_string(0, 4, txt);
                if (CFlag == 1)
                {
                    if (KEY_Read(KEY2) == 0)      //按下KEY2键，占空比增大
                    {
                        Ui_Stop_Speed = Ui_Stop_Speed + 1;
                        system_delay_ms(100);
                    }
                    if (KEY_Read(KEY4) == 0)
                    {
                        gpio_set_level(P20_3, 1);
                        gpio_set_level(P20_8, 1);
                        CFlag = 0;
                        system_delay_ms(100);
                        gpio_set_level(P20_3, 0);
                    }
                    if (KEY_Read(KEY1) == 0)      //按下KEY0键，占空比减小
                    {
                        Ui_Stop_Speed = Ui_Stop_Speed - 1;
                        system_delay_ms(100);
                    }
                }
                break;
            }

            default:
                break;
            }
        }

        //参数复位s1,s4
        if ((KEY_Read(KEY5) == 0))
        {
            sprintf(txt, "P: %05f;   ", Ui_Steer_P);
            oled_show_string(0, 0, txt);
            sprintf(txt, "D: %05f;   ", Ui_Steer_D);
            oled_show_string(0, 1, txt);
            sprintf(txt, "Line: %d;   ", Ui_Control_Line);
            oled_show_string(0, 2, txt);
            sprintf(txt, "Stra: %05f;   ", Ui_Straight_Diff_K);
            oled_show_string(0, 3, txt);
            sprintf(txt, "Bend: %05f;   ", Ui_Bend_Diff_K);
            oled_show_string(0, 4, txt);
            Flag_Stop = 0;
            CFlag = 0;
            system_delay_ms(100);
        }
        if ((KEY_Read(KEY6) == 0))
        {
            oled_clear();
            Ui_System();
            system_delay_ms(100);
        }
    }
}
void Ui_System(void)
{
    int T_Page_Turn = -1, Page_Turn = 0;//翻页标志1-3
    int T_S_Page_Turn = 0, S_Page_Turn = 0;//翻页确认标志0-1

    //UI调参初始化界面
    char txt[16];
    sprintf(txt, "SERVO_UI  ;");
    oled_show_string(0, 0, txt);
    sprintf(txt, "MOTOR_L   ;");
    oled_show_string(0, 1, txt);
    sprintf(txt, "MOTOR_R   ;");
    oled_show_string(0, 2, txt);
    sprintf(txt, "Fork      ;");
    oled_show_string(0, 3, txt);
    sprintf(txt, "RING      ;");
    oled_show_string(0, 4, txt);
    sprintf(txt, "SPEED     ;");
    oled_show_string(0, 5, txt);

    while (1)   //主循环
    {
        if (T_S_Page_Turn == 0)//选择信号 选择调节的参数 0 舵机 1 左电机 2 右电机
        {
            if (KEY_Read(KEY2) == 0)//向上选择按键
            {
                if (Page_Turn < 0)
                {
                    Page_Turn = 5;
                }
                else
                {
                    Page_Turn = Page_Turn - 1;
                    system_delay_ms(100);
                }
            }
            if (KEY_Read(KEY1) == 0)//向下选择按键
            {
                if (Page_Turn > 5)
                {
                    Page_Turn = 0;
                }
                else
                {
                    Page_Turn = Page_Turn + 1;
                    system_delay_ms(100);
                }
            }
            if (KEY_Read(KEY3) == 0)//确认按键
            {
                gpio_set_level(P20_3, 1);
                gpio_set_level(P20_8, 0);

                T_S_Page_Turn = 1;
                system_delay_ms(100);
                gpio_set_level(P20_3, 0);
            }
        }
        if (T_S_Page_Turn != S_Page_Turn || T_Page_Turn != Page_Turn)//选择信号确认后
        {
            T_Page_Turn = Page_Turn;
            switch (T_Page_Turn)
            {
            case 0:
            {
                sprintf(txt, "SERVO_UI<<;");
                oled_show_string(0, 0, txt);
                sprintf(txt, "MOTOR_L   ;");
                oled_show_string(0, 1, txt);
                sprintf(txt, "MOTOR_R   ;");
                oled_show_string(0, 2, txt);
                sprintf(txt, "Fork      ;");
                oled_show_string(0, 3, txt);
                sprintf(txt, "RING      ;");
                oled_show_string(0, 4, txt);
                sprintf(txt, "SPEED     ;");
                oled_show_string(0, 5, txt);
                if (T_S_Page_Turn == 1)
                {
                    system_delay_ms(100);
                    SteerUiSystem();
                }
                break;
            }
            case 1:
            {
                sprintf(txt, "SERVO_UI  ;");
                oled_show_string(0, 0, txt);
                sprintf(txt, "MOTOR_L <<;");
                oled_show_string(0, 1, txt);
                sprintf(txt, "MOTOR_R   ;");
                oled_show_string(0, 2, txt);
                sprintf(txt, "Fork      ;");
                oled_show_string(0, 3, txt);
                sprintf(txt, "RING      ;");
                oled_show_string(0, 4, txt);
                sprintf(txt, "SPEED     ;");
                oled_show_string(0, 5, txt);
                if (T_S_Page_Turn == 1)
                {
                    oled_clear();
                    system_delay_ms(100);
                    L_MotorUiSystem();
                }
                break;
            }
            case 2:
            {
                sprintf(txt, "SERVO_UI  ;");
                oled_show_string(0, 0, txt);
                sprintf(txt, "MOTOR_L   ;");
                oled_show_string(0, 1, txt);
                sprintf(txt, "MOTOR_R <<;");
                oled_show_string(0, 2, txt);
                sprintf(txt, "Fork      ;");
                oled_show_string(0, 3, txt);
                sprintf(txt, "RING      ;");
                oled_show_string(0, 4, txt);
                sprintf(txt, "SPEED     ;");
                oled_show_string(0, 5, txt);
                if (T_S_Page_Turn == 1)
                {
                    oled_clear();
                    system_delay_ms(100);
                    R_MotorUiSystem();
                }
                break;
            }
            case 3:
            {
                sprintf(txt, "SERVO_UI  ;");
                oled_show_string(0, 0, txt);
                sprintf(txt, "MOTOR_L   ;");
                oled_show_string(0, 1, txt);
                sprintf(txt, "MOTOR_R   ;");
                oled_show_string(0, 2, txt);
                sprintf(txt, "Fork    <<;");
                oled_show_string(0, 3, txt);
                sprintf(txt, "RING      ;");
                oled_show_string(0, 4, txt);
                sprintf(txt, "SPEED     ;");
                oled_show_string(0, 5, txt);
                if (T_S_Page_Turn == 1)
                {
                    oled_clear();
                    system_delay_ms(100);
                    Fork_UiSystem();
                }
                break;
            }
            case 4:
            {
                sprintf(txt, "SERVO_UI  ;");
                oled_show_string(0, 0, txt);
                sprintf(txt, "MOTOR_L   ;");
                oled_show_string(0, 1, txt);
                sprintf(txt, "MOTOR_R   ;");
                oled_show_string(0, 2, txt);
                sprintf(txt, "Fork      ;");
                oled_show_string(0, 3, txt);
                sprintf(txt, "RING    <<;");
                oled_show_string(0, 4, txt);
                sprintf(txt, "SPEED     ;");
                oled_show_string(0, 5, txt);
                if (T_S_Page_Turn == 1)
                {
                    oled_clear();
                    system_delay_ms(100);
                    Ring_UiSystem();
                }
                break;
            }
            case 5:
            {
                sprintf(txt, "SERVO_UI  ;");
                oled_show_string(0, 0, txt);
                sprintf(txt, "MOTOR_L   ;");
                oled_show_string(0, 1, txt);
                sprintf(txt, "MOTOR_R   ;");
                oled_show_string(0, 2, txt);
                sprintf(txt, "Fork      ;");
                oled_show_string(0, 3, txt);
                sprintf(txt, "RING      ;");
                oled_show_string(0, 4, txt);
                sprintf(txt, "SPEED   <<;");
                oled_show_string(0, 5, txt);
                if (T_S_Page_Turn == 1)
                {
                    oled_clear();
                    system_delay_ms(100);
                    Speed_UiSystem();
                }
                break;
            }
            default:
                break;
            }
            if ((KEY_Read(KEY4) == 0))
            {
                sprintf(txt, "SERVO_UI  ;");
                oled_show_string(0, 0, txt);
                sprintf(txt, "MOTOR_L   ;");
                oled_show_string(0, 1, txt);
                sprintf(txt, "MOTOR_R   ;");
                oled_show_string(0, 2, txt);
                sprintf(txt, "Fork      ;");
                oled_show_string(0, 3, txt);
                sprintf(txt, "RING      ;");
                oled_show_string(0, 4, txt);
                sprintf(txt, "SPEED   ;");
                oled_show_string(0, 5, txt);
                T_S_Page_Turn = 0;
                system_delay_ms(100);
            }
        }
        if ((KEY_Read(KEY5) == 0) && (KEY_Read(KEY6) == 0))
        {
            Ui_System();
        }
    }
}