#include <Ctrl.h>

/************************************************ 主 控 ************************************************/
void maincontrol(void)
{
    if (Flag_Garage_Out)
    {
        Update_PID(0, 0);
        Update_Angle(2, 0);
        return;
    }

    if (Flag_Garage_Out_Turn)
    {
        Update_PID(0, 0);
        if(!Flag_Garage_Turn_Dircection)
            Update_Angle(2, -60);
        if (Flag_Garage_Turn_Dircection)
            Update_Angle(2, 60);
        return;
    }

    if (Flag_Garage_In_Turn)
    {
        Update_PID(3, 0);
        if (!Flag_Garage_Turn_Dircection)
            Update_Angle(2, -60);
        if (Flag_Garage_Turn_Dircection)
            Update_Angle(2, 60);
        return;
    }

    if (ForkRoad)
    {
        if(ForkRoad_In)
        {
            Update_PID(0, 0);
            Update_Angle(4, 0);
            return;
        }

        if(ForkRoad_Inside)
        {
            Update_PID(4, 0);
            Update_Angle(3, 0);
            return;
        }
        if(ForkRoad_Out)
        {
            Update_PID(4, 0);
            Update_Angle(4, 0);
            return;
        }
    }

    if (Flag_T_Left_Lostline)
    {
        Update_PID(1, 0);
        Update_Angle(2, -60);
        return;
    }

    if (Flag_T_Right_Lostline)
    {
        Update_PID(1, 0);
        Update_Angle(2, 60);
        return;
    }

    if (L_Ring == 1 || R_Ring == 1)
    {
        Update_PID(1, 0);
        Update_Angle(1, 0);
        return;
    }

    if (Flag_Garage_In_Turn == 1)
    {
        Update_PID(3, 0);
        Update_Angle(2, -45);
        return;
    }

    Update_PID(0, 0);
    Update_Angle(0, 0);
}
/************************************************ 速 度 ************************************************/
int Fixed_Purpost_Speed = 0;                   //固定速度标志位
int OverMaxSpeedRoad_cnt = 0;                  //长直道计数
int Flag_OverMaxSpeed = 0;                     //长直道标志位
int Flag_OverMaxSpeed_Block = 0;              //屏蔽长直道标志位

/*更新目标速度（以EncSpeed为单位）*/
void Update_Purpost_Speed(void)
{
    //停车标志位处理
    if (Flag_Stop)
    {
        Purpost_Speed = 0;
        Purpost_Left_Speed = 0;
        Purpost_Rigt_Speed = 0;
        return;
    }

    if (Flag_Garage_Out_Turn)
    {
        Purpost_Left_Speed = Ui_Min_Speed - Ui_TSpeed_Change;
        Purpost_Rigt_Speed = Ui_Max_Speed + 40;
        return;
    }
    
    if (Flag_Garage_In_Turn)
    {
        Purpost_Left_Speed = Ui_Min_Speed - Ui_TSpeed_Change;
        Purpost_Rigt_Speed = Ui_Max_Speed + 40;
        return;
    }

    if (Flag_T_Left_Lostline)
    {
        Purpost_Left_Speed = Ui_Min_Speed - Ui_TSpeed_Change;
        Purpost_Rigt_Speed = Ui_Max_Speed  + 40;
        return;
    }

    if (Flag_T_Right_Lostline)
    {
        Purpost_Left_Speed = Ui_Max_Speed + 40;
        Purpost_Rigt_Speed = Ui_Min_Speed - Ui_TSpeed_Change;
        return;
    }

    short alpha = CurrentServoDty - Ui_Servo_Mid;   //当前打角（以ServoDuty为单位）
    if (!Fixed_Purpost_Speed)
        ChangeSpeed(alpha);
    else
        UnchangeSpeed(alpha);
}

//可变速度更新
void ChangeSpeed(sint16 alpha)
{
    //int i;
    //int temp_dev = 0, SpeedDown_Line = 0;
    //算出减速比
    float Decelerate_K = (Ui_Max_Speed - Ui_Min_Speed) / (float)Ui_Servo_Interval;
    /*目标速度的更新*/
    Purpost_Speed = Ui_Max_Speed - Decelerate_K * (float)abs(alpha);

    //长直道检测
   // LongStriaghtRoad(LoseCP_Cnt_f);

    //大弯道检测
//    for (i = 59; i > 3; i--)
//    {
//        if (imageLine1.Exist_Center[i])
//        {
//            temp_dev = imageLine1.Point_Center[i] - 45;
//            if (abs(temp_dev) > 20)
//            {
//                SpeedDown_Line = i;
//                break;
//            }
//        }
//    }
//    if (SpeedDown_Line > 10)
//    {
//        Purpost_Speed = Ui_Min_Speed;
//    }

    /*左右轮目标速度的更新*/
    if (!imageLine1.Lost_Center)
    {
        //差速
        if (abs(alpha) < Ui_Servo_Interval - 7)
        {
            if (alpha < 0)
            {
                //左转
                Purpost_Left_Speed = Purpost_Speed - Ui_Straight_Diff_K * (abs(alpha));
                Purpost_Rigt_Speed = Purpost_Speed;
            }
            else
            {
                //右转
                Purpost_Left_Speed = Purpost_Speed;
                Purpost_Rigt_Speed = Purpost_Speed - Ui_Straight_Diff_K * alpha;
            }
        }
        else
        {
            if (alpha < 0)
            {
                //左转时，左轮在内圈，右轮在外圈，所以左轮需要稍微减速
                //EncSpeed - Ui_Bend_Diff_K * ServoDuty
                //而Purpost_Left_Speed的单位是EncSpeed，所以Ui_Bend_Diff_K的单位应该是EncSpeed / ServoDuty
                Purpost_Left_Speed = Ui_Inner_Speed - Ui_Bend_Diff_K * (abs(alpha));
                Purpost_Rigt_Speed = Ui_Min_Speed;
            }
            else
            {
                //右转时，右轮在内圈，左轮在外圈，所以右轮需要稍微减速
                Purpost_Left_Speed = Ui_Min_Speed;
                Purpost_Rigt_Speed = Ui_Inner_Speed - Ui_Bend_Diff_K * alpha;
            }
        }
    }
    else
    {
        Purpost_Speed = Ui_Center_Lost_Speed;
        if (alpha < 0)
        {
            //左转
            Purpost_Left_Speed = Purpost_Speed - Ui_Bend_Diff_K * (abs(alpha));
            Purpost_Rigt_Speed = Purpost_Speed;
        }
        else
        {
            //右转
            Purpost_Left_Speed = Purpost_Speed;
            Purpost_Rigt_Speed = Purpost_Speed - Ui_Bend_Diff_K * alpha;
        }
    }
}
//固定速度更新
void UnchangeSpeed(sint16 alpha)
{
    if (alpha < 0) //左转时，左轮在内圈，右轮在外圈，所以左轮需要稍微减速
    {
        Purpost_Left_Speed = Purpost_Speed - Ui_Fixed_Diff_K * (abs(alpha));
        Purpost_Rigt_Speed = Purpost_Speed;
    }
    else           //右转时，右轮在内圈，左轮在外圈，所以右轮需要稍微减速
    {
        Purpost_Left_Speed = Purpost_Speed;
        Purpost_Rigt_Speed = Purpost_Speed - Ui_Fixed_Diff_K * alpha;
    }
}
//长直道判断
void LongStriaghtRoad(float cnt)
{
    if (cnt < 3)
    {
        OverMaxSpeedRoad_cnt++;
    }
    else if (cnt > 7)
    {
        Flag_OverMaxSpeed = 0;
        OverMaxSpeedRoad_cnt = 0;
    }

    if (OverMaxSpeedRoad_cnt > 50)
    {
        Flag_OverMaxSpeed = 1;
        Purpost_Speed = Ui_Max_Speed + 15;
    }
}
//设置固定速度
void SetToFixedSpeed(short SetFixed_Speed)
{
    Purpost_Speed = SetFixed_Speed;
    Fixed_Purpost_Speed = 1;
}
//取消固定速度
void CancelSetToFixedSpeed(void)
{
    Fixed_Purpost_Speed = 0;
}
/************************************************ 角度 ************************************************/
/*
  * @brief    更新目标角度
  * @param    pattern 模式   angle 固定角度
  * @return   Non
  * @note     pattern == 0   正常可变速度
  *           pattern == 1   圆环固定速度
  *           pattern == 2   正常可变速度（固定打角）
  *           pattern == 3   进三叉后固定速度
  *           pattern == 4   进三叉固定速度
  * @date     2022 5 9
  */
void Update_Angle(short pattern, int angle)
{
    if (pattern == 0)//正常可变速度
    {
        Servo_Duty = Ui_Servo_Mid + Servo_PID(&Steer_pid, Ui_Servo_Interval);
        Set_Servo_Duty(Servo_Duty);
        CancelSetToFixedSpeed();
    }
    if (pattern == 1)//圆环固定速度
    {
        Servo_Duty = Ui_Servo_Mid + Servo_PID(&Steer_pid, Ui_Servo_Interval);
        Set_Servo_Duty(Servo_Duty);
        SetToFixedSpeed(Ui_Ring_Speed);
    }
    if (pattern == 2)//正常可变速度（固定打角）
    {
        Deviation = angle;
        Servo_Duty = Ui_Servo_Mid + Servo_PID(&Steer_pid, Ui_Servo_Interval);
        Set_Servo_Duty(Servo_Duty);
        CancelSetToFixedSpeed();
    }
    if (pattern == 3)//进三叉后固定速度
    {
        Servo_Duty = Ui_Servo_Mid + Servo_PID(&Steer_pid, Ui_Servo_Interval);
        Set_Servo_Duty(Servo_Duty);
        SetToFixedSpeed(Ui_ForkRoad_Speed);
    }        
    if (pattern == 4)//进三叉固定速度
    {
        Servo_Duty = Ui_Servo_Mid + Servo_PID(&Steer_pid, Ui_Servo_Interval);
        Set_Servo_Duty(Servo_Duty);
        SetToFixedSpeed(Ui_Fork_Speed);
    }
}


/************************************************ PID ************************************************/

/*
  * @brief    更新pid
  * @param    pattern 模式   line 识别行
  * @return   Non
  * @note    （角度）
  *           pattern == 0   正常
  *           pattern == 1   圆环
  *           pattern == 2   坡道
  *           pattern == 4   三叉
  **          （速度）
  *           pattern == 3   停车
  * @date     2022 5 9
  */
void Update_PID(short pattern, int line)
{
    if (pattern == 0)//正常
    {
        Steer_pid.kp = Ui_Steer_P;
        Steer_pid.kd = Ui_Steer_D;
        Steer_pid.line = Ui_Control_Line;
    }
    if (pattern == 1)//圆环
    {
        Steer_pid.kp = Ui_Ring_P;
        Steer_pid.kd = Ui_Ring_D;
        Steer_pid.line = Ui_Control_Line + line;
    }
    if (pattern == 2)//坡道
    {
        Steer_pid.kp = 2.25f;
        Steer_pid.kd = 5.8f;
        Steer_pid.line = Ui_Control_Line + line;
    }
    if (pattern == 3)//停车（速度）
    {
        LSpeed_pid.kp = 200.0f;
        LSpeed_pid.ki = 15.0f;
        LSpeed_pid.kd = 8.0f;

        RSpeed_pid.kp = 200.0f;
        RSpeed_pid.ki = 15.0f;
        RSpeed_pid.kd = 8.0f;
    }
    if (pattern == 4)//三叉
    {
        Steer_pid.kp = Ui_Fork_P;
        Steer_pid.kd = Ui_Fork_D;
        Steer_pid.line = Ui_Control_Line + line;
    }
}

