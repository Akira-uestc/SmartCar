#ifndef _IMAGEPROCESS_H_
#define _IMAGEPROCESS_H_

#include <IfxCpu.h>
#include <Platform_Types.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <zf_device_mt9v03x.h>
#include <string.h>
#include "Duty.h"
#include "zf_driver_uart.h"
#include "zf_driver_timer.h"
#include "Variables.h"


#ifndef MIN
#  define MIN(a,b)  ((a) > (b) ? (b) : (a))
#endif

#ifndef MAX
#  define MAX(a,b)  ((a) < (b) ? (b) : (a))
#endif
#define ImageH    60  /*!< OLED显示高度（用户使用）高度 */
#define ImageW    94  /*!< OLED显示宽度（用户使用）宽度 */

#define K_MAX_THRESHOLD         10                      //两有效点间最大斜率
#define EFFECTIVE_ROW            3                      //定义图片最上层的有效行
#define MINRoadLen               4                      //最小路宽
#define AIM_LINE_SET            17                      //目标行

#define roadK                   (1.4)                   //直赛道K(14th)
#define roadB                   (10.45)                 //直赛道B
#define SingleLineLeanAveERR_MAX 45                     //一边丢线，未丢线一边最大平均倾斜误差
#define SingleLineLeanK          1                      //一边丢线，未丢线一边补偿系数(理论距离实际中线的距离)
#define LOST_LINE_THRESHOLD     55                      //丢边行最大值
#define VALID_LINE_THRESHOLE    60-LOST_LINE_THRESHOLD  //有效行最小值
#define CROSS_EDGE_LEN          4                       //拟合十字路口两段直线时所需的最少点数
/*图片信息*/
#define TOTAL_POINT = 16920;
//限幅
#define LIMIT2MAX(a,b) ((a) = (a)>(b)?(a):(b))
#define LIMIT2MIN(a,b) ((a) = (a)<(b)?(a):(b))
#define LIMIT(val,minV,maxV) ((val)=((val)>(maxV))?(maxV) : ( ((val)<(minV)) ? (minV) : (val)))

///*全局变量定义*/
///******************************包含图片所有有用信息**********************************/
typedef struct
{
    volatile int Exist_Left[60];                 //某一行左边边界点存在
    volatile int Exist_Right[60];                //某一行右边边界点存在
    volatile int Exist_Center[60];               //某一行中心点存在

    volatile int Lost_Center;                           //是否丢了中线
    volatile int Lost_Left;                             //是否丢了左线
    volatile int Lost_Right;                            //是否丢了右线

    volatile int Point_Left[60];                //存放某一行的左边界点
    volatile int Point_Right[60];               //存放某一行的右边界点
    volatile int Point_Center[60];              //存放某一行的中心点

    volatile short White_Num[60];                 //某一行的白点个数
} imageLine_t;                 //图片数据结构体定义
extern volatile imageLine_t imageLine;
extern volatile imageLine_t imageLine1;

//移植龙丘摄像头
extern unsigned char Camera_BIN;
#define LCDH    60  /*!< OLED显示高度（用户使用）高度 */
#define LCDW    94  /*!< OLED显示宽度（用户使用）宽度 */
/** 压缩后之后用于存放屏幕显示数据  */
unsigned char Image_Use[LCDH][LCDW];

/** 二值化后用于OLED显示的数据 */
unsigned char Bin_Image[LCDH][LCDW];
typedef unsigned short uint16;

/*数据传输*/
void ImageInformationDisplay(void);
void MasterComputerReport(void);
void SendMessage(void);

/*图像采集*/
void image(void);
void ImageProcessInit(void);
void ImageProcess_BIN(int i);
int GetOSTU_ln(unsigned char tmImage[LCDH][LCDW]);
void Binimage(int Threshold);
void BinImageFilter(void);
void Get_White_Num(void);
void trackDFS(void);
unsigned int isWhite(unsigned int row, unsigned int line);
unsigned int isLeftPoint(unsigned int i, unsigned int j);
unsigned int isRightPoint(unsigned int i, unsigned int j);
int LeftLine_Check(int EndLine);
int RightLine_Check(int EndLine);

void ImageProcessInit1(void);
void mediumLineCheck(void);
void ImageProcess_EdgeLine(void);
void ImageProcess_MidLine(void);


/*滤波*/
void Left_Right_Confusion_Filter(void);
void left_right_Limit(void);
void doFilter(void);
void lineChangeLimit(void);
void lostLine_Filter(void);
void position_Filter(void);
void slope_Filter(void);
void singlePoint_Filter(void);

void Ring_Fliter(void);

/*补线*/
void doMend(void);
void StraightLineJudge(void);
void trackMend_startPart(void);
void trackMend_HalfWidth(void);
void trackMend_endPart(void);
int isEdgePoint(int i, int j);
void leastSquareMethod(short* x, short* y, int len, float* k, float* b);
void repairRemainLine(void);
float getLeastSquareMethodERROR(short* x, short* y, int len, float k, float b);
void amplitudeLIMIT(int i, int amp);
void limitCenter(void);

/*********************************************************出入库*********************************************************/
extern int Flag_Garage_Out_Turn;
extern int Flag_Garage_In_Find;
extern int Flag_Garage_In_Cnt;
extern int Flag_Garage_In_Turn;

void Garage_In(void);
void Garage_Out(void);
/*****************************************************三叉************************************************************************/
extern short isForkRoadTurnLeft;
extern short Fork_Cnt;
extern short ForkRoad_Cnt;
extern short ForkRoad;
extern short ForkRoad_Inside;
extern short ForkRoad_In;
extern short ForkRoad_Out;

void ForkMend(void);
void TiltFork_FliterL(void);
void TiltFork_FliterR(void);
void OutsideFork_Mend(void);
void InsideFork_Mend(void);
void ForkRoad_States(void);
/*****************************************************十字************************************************************************/
void Cross_Filter(void);
void CrossFliter_Left(void);
void CrossFliter_Right(void);
void CrossMend_Right(void);
void CrossMend_Left(void);
/*****************************************************圆环************************************************************************/
extern int L_Ring;
extern int R_Ring;

extern int Flag_Left_Ring_OnlyOnce;
extern int Flag_Left_Ring_Clc;
extern int Flag_Right_Ring_OnlyOnce;
extern int Flag_Right_Ring_Clc;

void Left_Ring_Find(void);
void Left_Ring_Turn(void);
void Left_Ring_Out(void);
void Left_Ring_Out_Mend(void);
void Left_Ring(void);

void Right_Ring_Find(void);
void Right_Ring_Turn(void);
void Right_Ring_Out(void);
void Right_Ring_Out_Mend(void);
void Right_Ring(void);
/*****************************************************往返************************************************************************/
extern int T_Left_Road;
extern int Flag_T_Left_Lostline;
extern int Flag_T_Left_OnlyOnce;
extern int Flag_T_Left_Clc;
extern int T_Right_Road;
extern int Flag_T_Right_Lostline;
extern int Flag_T_Right_OnlyOnce;
extern int Flag_T_Right_Clc;

void T_Left(void);
void T_Left_Find(void);
void T_Left_Turn(void);
void T_Left_Out(void);
void T_Right(void);
void T_Right_Find(void);
void T_Right_Turn(void);
void T_Right_Out(void);
/***************************************************坡道************************************************************************/
extern uint16 TF_distance;             // TF_测距数据
extern uint16 TF_strength;             // TF_信号强度
extern uint16 TF_temperature;          // TF_温度

void TF_Uart(void);
/**************************************************中线误差************************************************************************/
extern int Deviation;
extern float LoseCP_Cnt_f;
void Get_MidlineErr(void);

#endif
