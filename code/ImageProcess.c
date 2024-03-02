#include <ImageProcess.h>

volatile imageLine_t imageLine;
volatile imageLine_t imageLine1;
static short stackTopPos; //栈顶指针
static unsigned short Threshold = 0; //定义二值化阈值
static unsigned int roadPointStackX[100]; //DFS堆栈存储寻找X坐标
static unsigned int roadPointStackY[100]; //DFS堆栈存储寻找Y坐标
static int isVisited[60][94];
int Start_time = 0;
int End_time = 0;

static int isLeftLineStraight = 1; //左线是否为直线
static int isRightLineStraight = 1; //右线是否为直线

static int isLeftRightLineExist = 1; //降标准检测左右边界是否存在
volatile int center = 30;
volatile int AIM_LINE = AIM_LINE_SET; //目标行（本行所找中点作为的参考行）

//三叉
short isForkRoadTurnLeft = 1; //三岔转向方向 1 左转; 0 右转
short Flag_ForkRoad_Mend = 0;
short Fork_Cnt = 0;
short ForkRoad_Cnt = 0;
short ForkRoad = 0;//三叉全程 用与屏蔽其他元素处理
short ForkRoad_Inside = 0;//三叉内
short ForkRoad_In = 0;//进三叉路口
short ForkRoad_Out = 0;//出三叉路口

//圆环
int Flag_Left_Ring_Find = 0;
int Flag_Left_Ring_Turn = 0;
int Flag_Left_Ring_Out = 0;
int Flag_Left_Ring_Out_Mend = 0;
int Flag_Left_Ring_OnlyOnce = 1;
int Flag_Left_Ring_Clc = 0;
int L_Ring = 0;

int Flag_Right_Ring_Find = 0;
int Flag_Right_Ring_Turn = 0;
int Flag_Right_Ring_Out = 0;
int Flag_Right_Ring_Out_Mend = 0;
int Flag_Right_Ring_OnlyOnce = 1;
int Flag_Right_Ring_Clc = 0;
int R_Ring = 0;
//往返
int Flag_T_Left_Find = 0;
int Flag_T_Left_Turn = 0;
int Flag_T_Left_Out = 0;
int Flag_T_Left_Lostline = 0;
int Flag_T_Left_OnlyOnce = 1;
int Flag_T_Left_Clc = 0;
int T_Left_Road = 0;

int Flag_T_Right_Find = 0;
int Flag_T_Right_Turn = 0;
int Flag_T_Right_Out = 0;
int Flag_T_Right_Lostline = 0;
int Flag_T_Right_OnlyOnce = 1;
int Flag_T_Right_Clc = 0;
int T_Right_Road = 0;
//出入库
int Flag_Garage_Out_Turn = 0;
int Flag_Garage_In_Turn = 0;
int Flag_Garage_In_Cnt = 0;
int Flag_Garage_In_Find = 0;
//坡道
uint16 TF_distance = 0;             // TF_测距数据
uint16 TF_strength = 0;             // TF_信号强度
uint16 TF_temperature = 0;          // TF_温度
uint16 TF_checksum = 0;             // TF_自检
uint8 TF_dat[9];
uint8 TF_num = 0;

//计算中线偏差
int Deviation = 0;
float LoseCP_Cnt_f = 50;


void ImageProcess_EdgeLine(void)
{
    /*边界点检索*/
    trackDFS();
    left_right_Limit();    //左右边界限制
    lineChangeLimit();    //边界斜率限制
    singlePoint_Filter();    //滤除单个边界点
    slope_Filter();        //滤除斜率不对的边界点
    position_Filter();     //滤除位置不对的点
    lostLine_Filter();     //判断是否丢边线
}
void ImageProcess_MidLine(void)
{
    trackMend_startPart();
    trackMend_HalfWidth();
    trackMend_endPart();
}
void ImageProcessInit1(void)
{
    int i, j;
    //边线和中心线都不存在
    imageLine1.Lost_Center = 1;
    imageLine1.Lost_Left = 1;
    imageLine1.Lost_Right = 1;
    stackTopPos = -1; //栈顶指针初值
    for (i = 0; i < ImageH; i++)
    {
        //每一行的左右边界点和中心点都不存在
        imageLine1.Exist_Left[i] = 0;
        imageLine1.Exist_Right[i] = 0;
        imageLine1.Exist_Center[i] = 0;

        //边界点和中心点设为初始位置
        imageLine1.Point_Left[i] = 1;
        imageLine1.Point_Right[i] = ImageW - 1;
        imageLine1.Point_Center[i] = ImageW / 2;

        //每一行的白点个数清零
        imageLine1.White_Num[i] = 0;

        for (j = 0; j < ImageW; j++)
        {
            isVisited[i][j] = 0; //DFS用, 所有点都还没被遍历过
        }
    }
}
void mediumLineCheck(void)
{
    int i;

    if (!imageLine.Lost_Center)    //首先中线没丢
    {
        //(一) 右丢线
        if (imageLine.Lost_Right)
        {
            //图像上半部分左线未超过阈值且左线未越过中间则补线无效
            int leftMAX = 0;    //表示左边界点横向最右延伸到了哪里
            int leftCount = 0;

            for (i = 0; i < ImageH * 2 / 3; i++)    ///////////////////////////////////////////////
            {
                if (imageLine.Exist_Left[i])
                {
                    leftCount++;
                    if (imageLine.Point_Left[i] > leftMAX)
                        leftMAX = imageLine.Point_Left[i];
                }
            }

            //if ((leftCount < MT9V03X_H / 6) && (leftMAX < MT9V03X_W / 3))
            if ((leftCount < ImageH / 6) && (leftMAX < ImageW))
            {
                imageLine.Lost_Center = 1;
            }

        }

        //(二) 左丢线
        else if (imageLine.Lost_Left)
        {
            //图像上半部分右线未超过阈值且右线未越过中间则补线无效
            int rightMIN = ImageW;    //表示右边界点横向最左延伸到了哪里
            int rightCount = 0;

            for (i = 0; i < ImageH * 2 / 3; i++)
            {
                if (imageLine.Exist_Right[i])
                {
                    rightCount++;
                    if (imageLine.Point_Right[i] < rightMIN)
                        rightMIN = imageLine.Point_Right[i];
                }
            }

            //if ((rightCount < MT9V03X_H / 6) && (rightMIN > MT9V03X_W * 2 / 3))
            if ((rightCount < ImageH / 6) && (rightMIN > 1))
            {
                imageLine.Lost_Center = 1;
            }
        }

        //(三) 不丢线
        else
        {
            int len_temp;
            int len_basis;
            int lostCenter_cnt = 0;

            for (i = ImageH / 2; i < ImageH; i++)    //从图像中间开始向下检测
            {
                //如果本行左右边点存在
                if (imageLine.Exist_Left[i] && imageLine.Exist_Right[i])
                {
                    len_temp = imageLine.Point_Right[i] - imageLine.Point_Left[i];
                    len_basis = roadK * i + roadB;

                    //本行实际路宽应全满或大于len_basis - 10
                    //如不满足, 无效中线并跳出循环
                    if ((len_temp < ImageW - 5) && (len_temp < len_basis - 15))
                    {
                        lostCenter_cnt++;
                        //imageLine.Lost_Center = 1;
                        //break;
                    }

                    if (lostCenter_cnt > 5)
                    {
                        lostCenter_cnt = 0;
                        imageLine.Lost_Center = 1;
                        break;
                    }
                }
            }

        }

    }
    else
        //中线要是丢了校验寂寞呢
        return;
}

/****************************************************数据传输***********************************************************************/
/*图片信息输出
void ImageInformationDisplay(void)
{
    char txt[32];

    sprintf(txt, "Find=%d;", Flag_Left_Ring_Find);
    OLED_P6x8Str(0, 0, txt);
    sprintf(txt, "Turn=%d;", Flag_Left_Ring_Turn);
    OLED_P6x8Str(0, 1, txt);
    sprintf(txt, "Out=%d;", Flag_Left_Ring_Out);
    OLED_P6x8Str(0, 2, txt);
    sprintf(txt, "Mend=%d;", Flag_Left_Ring_Out_Mend);
    OLED_P6x8Str(0, 3, txt);
}


//图传函数
void MasterComputerReport(void)
{
    if (mt9v03x_finish_flag == 1)
    {
        ZW_Send_Image(Bin_Image);
        mt9v03x_finish_flag = 0;
    }
}
//串口传输
void SendMessage(void)
{
    char txt[32];
    sprintf(txt, "%f,%d,%d,%f,%d,%d,%d\n", LSpeed_pid.pid_out, LEnc_Val, Purpost_Left_Speed, RSpeed_pid.pid_out, REnc_Val, Purpost_Rigt_Speed, Deviation);
    UART_PutStr(UART0, txt);
}*/


/****************************************************图像采集***********************************************************************/
/*图像采集合集*/
void image(void)
{
    /* 提取部分使用的数据 */
    Get_Use_Image();
    /* 清除摄像头采集完成标志位  如果不清除，则不会再次采集数据 */
    mt9v03x_finish_flag = 0;

    ImageProcessInit(); //图片信息初始化
    /* 二值化 */
    ImageProcess_BIN(0); //0不使用上位机，1使用
    Get_White_Num();
    trackDFS();
}
/*图片信息初始化*/
void ImageProcessInit(void)
{
    int i, j;
    //边线和中心线都不存在
    imageLine.Lost_Center = 1;
    imageLine.Lost_Left = 1;
    imageLine.Lost_Right = 1;

    stackTopPos = -1;//栈顶指针初值

    for (i = 0; i < ImageH; i++)
    {
        //每一行的左右边界点和中心点都不存在
        imageLine.Exist_Left[i] = 0;
        imageLine.Exist_Right[i] = 0;
        imageLine.Exist_Center[i] = 0;

        //边界点和中心点设为初始位置
        imageLine.Point_Left[i] = 1;
        imageLine.Point_Right[i] = ImageW - 1;
        imageLine.Point_Center[i] = ImageW / 2;

        //每一行的白点个数清零
        imageLine.White_Num[i] = 0;


        for (j = 0; j < ImageW; j++)
        {
            isVisited[i][j] = 0;//DFS用, 所有点都还没被遍历过
        }
    }
}
/*图像二值化*/
void ImageProcess_BIN(int i)
{

    if (i == 0)
    {
        /*计算阈值*/
        Threshold = (unsigned short)GetOSTU_ln(Image_Use);
        Binimage(Threshold);
        mt9v03x_finish_flag = 1;
    }
    else
    {
        Get_Bin_Image(0);
        mt9v03x_finish_flag = 1;
    }
    BinImageFilter();
}
/*大津法*/
int GetOSTU_ln(unsigned char tmImage[LCDH][LCDW])
{
    signed short i, j;
    unsigned long Amount = 0;
    unsigned long PixelBack = 0;
    unsigned long PixelshortegralBack = 0;
    unsigned long Pixelshortegral = 0;
    signed long PixelshortegralFore = 0;
    signed long PixelFore = 0;
    float OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // 类间方差;
    signed short MinValue, MaxValue;
    int Threshold = 0;
    unsigned char HistoGram[256];              //

    for (j = 0; j < 256; j++)
        HistoGram[j] = 0; //初始化灰度直方图

    for (j = 0; j < ImageH; j++)
    {
        for (i = 0; i < ImageW; i++)
        {
            HistoGram[tmImage[j][i]]++; //统计灰度级中每个像素在整幅图像中的个数
        }
    }

    for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++)
        ;        //获取最小灰度的值
    for (MaxValue = 255; MaxValue > MinValue && HistoGram[MinValue] == 0; MaxValue--)
        ; //获取最大灰度的值

    if (MaxValue == MinValue)
        return MaxValue;         // 图像中只有一个颜色
    if (MinValue + 1 == MaxValue)
        return MinValue;        // 图像中只有二个颜色

    for (j = MinValue; j <= MaxValue; j++)
        Amount += HistoGram[j];        //  像素总数

    Pixelshortegral = 0;
    for (j = MinValue; j <= MaxValue; j++)
    {
        Pixelshortegral += HistoGram[j] * j;        //灰度值总数
    }
    SigmaB = -1;
    for (j = MinValue; j < MaxValue; j++)
    {
        PixelBack = PixelBack + HistoGram[j];     //前景像素点数
        PixelFore = Amount - PixelBack;           //背景像素点数
        OmegaBack = (float)PixelBack / Amount;   //前景像素百分比
        OmegaFore = (float)PixelFore / Amount;   //背景像素百分比
        PixelshortegralBack += HistoGram[j] * j;  //前景灰度值
        PixelshortegralFore = Pixelshortegral - PixelshortegralBack;  //背景灰度值
        MicroBack = (float)PixelshortegralBack / PixelBack;   //前景灰度百分比
        MicroFore = (float)PixelshortegralFore / PixelFore;   //背景灰度百分比
        Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);   //计算类间方差
        if (Sigma > SigmaB)                    //遍历最大的类间方差g //找出最大类间方差以及对应的阈值
        {
            SigmaB = Sigma;
            Threshold = j;
        }
    }
    return Threshold;                        //返回最佳阈值;
}
/*根据阈值将图片二值化*/
void Binimage(int Threshold)
{
    for (int i = 0; i < ImageH; i++)
    {
        for (int j = 0; j < ImageW; j++)
        {
            if (Image_Use[i][j] > Threshold) //数值越大，显示的内容越多，较浅的图像也能显示出来
                Bin_Image[i][j] = 255;
            else
                Bin_Image[i][j] = 0;
        }
    }
}
/*噪点过滤*/
void BinImageFilter(void)
{
    int nr; //行
    int nc; //列

    for (nr = 1; nr < ImageH - 1; nr++)
    {
        for (nc = 1; nc < ImageW - 1; nc = nc + 1)
        {
            if ((Bin_Image[nr][nc] == 0) && (Bin_Image[nr - 1][nc] + Bin_Image[nr + 1][nc] + Bin_Image[nr][nc + 1] + Bin_Image[nr][nc - 1] > 510))
            {
                Bin_Image[nr][nc] = 255;
            }
            else if ((Bin_Image[nr][nc] == 255) && (Bin_Image[nr - 1][nc] + Bin_Image[nr + 1][nc] + Bin_Image[nr][nc + 1] + Bin_Image[nr][nc - 1] < 510))
            {
                Bin_Image[nr][nc] = 0;
            }
        }
    }
}
/*判断每一行的黑白像素点数*/
void Get_White_Num(void)
{
    short i, j;
    short white_num = 0;

    for (i = 0; i < ImageH; i++)
    {
        for (j = 0; j < ImageW; j++)
        {
            if (Bin_Image[i][j])
            {
                white_num++;
            }
        }
        imageLine.White_Num[i] = white_num;
        white_num = 0;
    }

}
/*边界点检索*/
void trackDFS(void)
{
    int i, j;

    //选择图片下方中点作为起始点
    if (isWhite(ImageH - 2, ImageW / 2) && Flag_Left_Ring_Turn == 0 && Flag_Right_Ring_Turn == 0)//若下方中点就是白点
    {
        stackTopPos++;
        roadPointStackX[stackTopPos] = ImageH - 2;
        roadPointStackY[stackTopPos] = ImageW / 2;
        isVisited[ImageH - 2][ImageW / 2] = 1;
    }
    else
    {
        for (i = 0; i <= ImageW / 2; i++)//从右往左搜索
        {
            if (Flag_Left_Ring_Turn == 0 && Flag_Right_Ring_Turn == 0)
            {
                if (isWhite(ImageH - 2, ImageW / 2 - i - 2) && isWhite(ImageH - 2, ImageW / 2 - i - 1) && isWhite(ImageH - 2, ImageW / 2 - i)
                    && isWhite(ImageH - 2, ImageW / 2 - i + 1) && isWhite(ImageH - 2, ImageW / 2 - i + 2)//连续5个白点
                    )
                {
                    //搜索到就入栈
                    stackTopPos++;//stackTopPos非零就表示栈非空
                    roadPointStackX[stackTopPos] = ImageH - 2;
                    roadPointStackY[stackTopPos] = i;
                    isVisited[ImageH - 2][i] = 1;
                    break;
                }
                if (isWhite(ImageH - 2, ImageW / 2 + i - 2) && isWhite(ImageH - 2, ImageW / 2 + i - 1) && isWhite(ImageH - 2, ImageW / 2 + i)
                    && isWhite(ImageH - 2, ImageW / 2 + i + 1) && isWhite(ImageH - 2, ImageW / 2 + i + 2)//连续5个白点
                    )
                {
                    //搜索到就入栈
                    stackTopPos++;//stackTopPos非零就表示栈非空
                    roadPointStackX[stackTopPos] = ImageH - 2;
                    roadPointStackY[stackTopPos] = i;
                    isVisited[ImageH - 2][i] = 1;
                    break;
                }
            }
            else
            {
                if (Flag_Left_Ring_Turn)
                {
                    if (isWhite(ImageH - 2, i - 2) && isWhite(ImageH - 2, i - 1) && isWhite(ImageH - 2, i)
                        && isWhite(ImageH - 2, i + 1) && isWhite(ImageH - 2, i + 2)//连续5个白点
                        )
                    {
                        //搜索到就入栈
                        stackTopPos++;//stackTopPos非零就表示栈非空
                        roadPointStackX[stackTopPos] = ImageH - 2;
                        roadPointStackY[stackTopPos] = i;
                        isVisited[ImageH - 2][i] = 1;

                    }
                }
                else
                {
                    if (isWhite(ImageH - 2, ImageW - i - 2) && isWhite(ImageH - 2, ImageW - i - 1) && isWhite(ImageH - 2, ImageW - i)
                        && isWhite(ImageH - 2, ImageW - i + 1) && isWhite(ImageH - 2, ImageW - i + 2)//连续5个白点
                        )
                    {
                        //搜索到就入栈
                        stackTopPos++;//stackTopPos非零就表示栈非空
                        roadPointStackX[stackTopPos] = ImageH - 2;
                        roadPointStackY[stackTopPos] = i;
                        isVisited[ImageH - 2][i] = 1;

                    }
                }
            }
        }
    }

    i = j = 0;

    while (stackTopPos >= 0)
    {
        //出栈
        i = roadPointStackX[stackTopPos];
        j = roadPointStackY[stackTopPos];
        stackTopPos--;

        //处理出界，直接continue
        if ((i < 3) || (i > ImageH - 2) || (j < 1) || (j > ImageW - 2))
        {
            continue;
        }

        /*************以下操作原则是：遇白点入栈，遇黑点初步判断为边界点(后续还需要修正滤波补线等操作)**************/
        //堆栈存储：...左右(上)左右(上)...
        //一般情况下，上的白点会先出栈(体现为先一直往图像上方搜索)，然后再出左右
        //向左搜索
        if (!isVisited[i][j - 1])
        {
            if (isWhite(i, j - 1))
            {
                //白点入栈
                stackTopPos++;
                roadPointStackX[stackTopPos] = i;
                roadPointStackY[stackTopPos] = j - 1;
                isVisited[i][j - 1] = 1;
            }
            else
            {
                //黑点初步判断为边界点(xzl: 感觉会出现覆盖的情况)
                if (isLeftPoint(i, j))
                {
                    imageLine.Point_Left[i] = j;//左线轨迹
                    imageLine.Exist_Left[i] = 1;
                    imageLine.Lost_Left = 0;
                }
            }
        }

        //向右搜索
        if (!isVisited[i][j + 1])
        {
            if (isWhite(i, j + 1))
            {
                stackTopPos++;
                roadPointStackX[stackTopPos] = i;
                roadPointStackY[stackTopPos] = j + 1;
                isVisited[i][j + 1] = 1;
            }
            else//可能找到右边界
            {
                if (isRightPoint(i, j))
                {
                    imageLine.Point_Right[i] = j;//右线轨迹
                    imageLine.Exist_Right[i] = 1;
                    imageLine.Lost_Right = 0;
                }
            }
        }
        //向上搜索(向上不判断边界点)
        if (!isVisited[i - 1][j])
        {
            if (isWhite(i - 1, j))
            {
                stackTopPos++;
                roadPointStackX[stackTopPos] = i - 1;
                roadPointStackY[stackTopPos] = j;
                isVisited[i - 1][j] = 1;
            }
        }
    }
}
unsigned int isWhite(unsigned int row, unsigned int line)
{
    //出界判断
    if (!(row >= 0 && row < ImageH && line >= 0 && line < ImageW))
        return 0;
    //判断白点黑点
    if (Bin_Image[row][line])
        return 1;
    else
        return 0;
}
/*边界点判断*/
unsigned int isLeftPoint(unsigned int i, unsigned int j)
{
    if (j < 1 || j >= ImageW || i<0 || i>ImageH)//图像边缘
        return 0;
    //右边一定不能出现蓝布
    if (((!isWhite(i, j)) || (!isWhite(i, j + 1)) || (!isWhite(i, j + 2)) || (!isWhite(i, j + 3)) || (!isWhite(i, j + 4)) || (!isWhite(i, j + 5))))
        return 0;
    //左边一定不能出现路
    if (isWhite(i, j - 1) || isWhite(i, j - 2) || isWhite(i, j - 3) || isWhite(i, j - 4) || isWhite(i, j - 5))
        return 0;
    return 1;
}
unsigned int isRightPoint(unsigned int i, unsigned int j)
{
    if (j < 2 || j >= ImageW || i<0 || i>ImageH)//图像边缘
        return 0;
    //左边一定不能出现蓝布
    if (((!isWhite(i, j)) || (!isWhite(i, j - 1)) || (!isWhite(i, j - 2)) || (!isWhite(i, j - 3)) || (!isWhite(i, j - 4)) || (!isWhite(i, j - 5))))
        return 0;
    //右边一定不能出现路
    if (isWhite(i, j + 1) || isWhite(i, j + 2) || isWhite(i, j + 3) || isWhite(i, j + 4) || isWhite(i, j + 5))
        return 0;

    return 1;
}
/*边界线是否为直线判断*/
int LeftLine_Check(int EndLine)
{
    int i = 0, j = 0;
    int StartLine = 0;
    //找到右边线起始点
    for (i = 59; i > 30; i--)
    {
        if (imageLine.Exist_Left[i])
        {
            StartLine = i;
            break;
        }
        else
            continue;
    }
    if (StartLine == 0)
    {
        return 0;
    }
    //判断右线位置是否正常
    int LeftLineMAX = 0;
    for (i = StartLine; i > EndLine; i--)
    {
        if (imageLine.Exist_Left[i] && (imageLine.Point_Left[i] > LeftLineMAX))
        {
            LeftLineMAX = imageLine.Point_Left[i];
        }
    }
    if (LeftLineMAX < 10 || LeftLineMAX >54)
    {
        return 0;
    }
    //判断右边界点的数量是否正常
    int LeftLine_Cnt = 0;
    for (i = StartLine; i > EndLine; i--)
    {
        if (imageLine.Exist_Left[i])
        {
            LeftLine_Cnt++;
        }
    }
    if (LeftLine_Cnt < 4 * (StartLine - EndLine) / 5)
    {
        return 0;
    }
    //判断右边线斜率是否正常
    int LeftK = 0;
    for (i = StartLine; i > EndLine; i--)    //从下往上
    {
        if (imageLine.Exist_Left[i])    //先找到第一个有效点
        {
            for (j = i - 1; j > i - 5; j--)    //再向下找临近有效点
            {
                if (imageLine.Exist_Left[j])
                {
                    LeftK = getLineK(j, imageLine.Point_Left[j], i, imageLine.Point_Left[i]);
                    if (LeftK < -2 || LeftK > 0)
                    {
                        return 0;
                    }
                    break;    //只要找到一个临近有效点，检测后就break到下一个i
                }
                else
                    continue;
            }
        }
        else
            continue;
    }
    return 1;
}
int RightLine_Check(int EndLine)
{
    int i = 0, j = 0;
    int StartLine = 0;
    //找到右边线起始点
    for (i = 59; i > 30; i--)
    {
        if (imageLine.Exist_Right[i])
        {
            StartLine = i;
            break;
        }
        else
            continue;
    }
    if (StartLine == 0)
    {
        return 0;
    }
    //判断右线位置是否正常
    int rightLineMAX = 94;
    for (i = StartLine; i > EndLine; i--)
    {
        if (imageLine.Exist_Right[i] && (imageLine.Point_Right[i] < rightLineMAX))
        {
            rightLineMAX = imageLine.Point_Right[i];
        }
    }
    if (rightLineMAX < 40 || rightLineMAX >80)
    {
        return 0;
    }
    //判断右边界点的数量是否正常
    int RightLine_Cnt = 0;
    for (i = StartLine; i > EndLine; i--)
    {
        if (imageLine.Exist_Right[i] && imageLine.Point_Right[i] != 94)
        {
            RightLine_Cnt++;
        }
    }
    if (RightLine_Cnt < 4 * (StartLine - EndLine) / 5)
    {
        return 0;
    }
    //判断右边线斜率是否正常
    int rightK = 0;
    for (i = StartLine; i > EndLine; i--)    //从下往上
    {
        if (imageLine.Exist_Right[i])    //先找到第一个有效点
        {
            for (j = i - 1; j > i - 5; j--)    //再向下找临近有效点
            {
                if (imageLine.Exist_Right[j])
                {
                    rightK = getLineK(j, imageLine.Point_Right[j], i, imageLine.Point_Right[i]);
                    if (rightK > 2 || rightK < 0)
                    {
                        return 0;
                    }
                    break;    //只要找到一个临近有效点，检测后就break到下一个i
                }
                else
                    continue;
            }
        }
        else
            continue;
    }
    return 1;
}
/*****************************************************滤波************************************************************************/
/*深度遍历滤波*/
void Left_Right_Confusion_Filter(void)
{
    int i = 0, j = 0;
    for (i = 59; i >= 0; i--)
    {
        if (imageLine.Exist_Right[i] && imageLine.Exist_Left[i])
        {
            if (imageLine.Point_Left[i] > imageLine.Point_Right[i])
            {
                for (j = imageLine.Point_Left[i] + 1; j < 93; j++)
                {
                    if (!isWhite(i, j + 1))
                    {
                        if (isRightPoint(i, j))
                            imageLine.Point_Right[i] = j;
                        break;
                    }
                }
                for (j = imageLine.Point_Right[i] - 1; j > 0; j--)
                {
                    if (!isWhite(i, j - 1))
                    {
                        if (isLeftPoint(i, j))
                            imageLine.Point_Left[i] = j;
                        break;
                    }
                }
            }
        }
    }
    for (i = 59; i > 0; i--)
    {
        if (imageLine.Exist_Right[i] && imageLine.Exist_Right[i - 1] && (abs(imageLine.Point_Right[i] - imageLine.Point_Right[i - 1])) > 5)
        {
            for (j = 5; j > 0; j--)
            {
                if (isRightPoint(i - 1, imageLine.Point_Right[i] + j))
                {
                    imageLine.Point_Right[i - 1] = imageLine.Point_Right[i - 1] + j;
                    break;
                }

                if (isRightPoint(i - 1, imageLine.Point_Right[i - 1] - j))
                {
                    imageLine.Point_Right[i - 1] = imageLine.Point_Right[i - 1] - j;
                    break;
                }
            }
        }
        if (imageLine.Exist_Left[i] && imageLine.Exist_Left[i - 1] && (abs(imageLine.Exist_Left[i] - imageLine.Exist_Left[i - 1])) > 5)
        {
            for (j = 5; j > 0; j--)
            {
                if (isLeftPoint(i - 1, imageLine.Point_Left[i] + j))
                {
                    imageLine.Point_Left[i - 1] = imageLine.Point_Left[i - 1] + j;
                    break;
                }

                if (isLeftPoint(i - 1, imageLine.Point_Left[i - 1] - j))
                {
                    imageLine.Point_Left[i - 1] = imageLine.Point_Left[i - 1] - j;
                    break;
                }
            }
        }
    }

}
//右边界不在左边界左边
void left_right_Limit(void)
{
    short i = 0;

    for (i = 1; i < ImageH - 1; i++)//从左上到右下
    {
        if (imageLine.Exist_Left[i] && imageLine.Exist_Right[i])
        {
            if (imageLine.Point_Left[i] > imageLine.Point_Right[i])
            {
                if (Flag_Right_Ring_Out)
                {
                    imageLine.Exist_Right[i] = 0;
                    continue;
                }
                if (Flag_Left_Ring_Out)
                {
                    imageLine.Exist_Left[i] = 0;
                    continue;
                }
                imageLine.Exist_Right[i] = 0;
                imageLine.Exist_Left[i] = 0;
            }
        }
    }
}

/*滤波合集*/
void doFilter(void)
{
    lineChangeLimit();    //边界斜率限制
    lostLine_Filter();    //无效行过多滤去(丢边线判断)
    position_Filter();    //位置不对滤去
    slope_Filter();    //斜率不对滤去
    lostLine_Filter();
    //存疑
    singlePoint_Filter();    //测试：单独点滤去
}
/*边界线不突变*/
void lineChangeLimit(void)
{
    short i, j;
    float leftK = 0;
    float rightK = 0;

    //左边界相邻两有效点斜率检测
    for (i = ImageH - 2; i > 0; i--)        //从下往上
    {
        if (imageLine.Exist_Left[i])        //先找到第一个有效点
        {
            for (j = i - 1; j > 0; j--)        //再向下找临近有效点
            {
                if (imageLine.Exist_Left[j])
                {
                    leftK = getLineK(i, imageLine.Point_Left[i], j, imageLine.Point_Left[j]);

                    if (abs(leftK) > K_MAX_THRESHOLD)
                    {
                        imageLine.Exist_Left[j] = 0;
                        continue;
                    }
                    break;        //只要找到一个临近有效点，检测后就break到下一个i
                }
                else
                    continue;
            }
        }
        else
            continue;
    }

    //右边界相邻两有效点斜率检测
    for (i = ImageH - 2; i > 0; i--)        //从下往上
    {
        if (imageLine.Exist_Right[i])        //先找到第一个有效点
        {
            for (j = i - 1; j > 0; j--)        //再向下找临近有效点
            {
                if (imageLine.Exist_Right[j])
                {
                    rightK = getLineK(i, imageLine.Point_Right[i], j, imageLine.Point_Right[j]);

                    if (abs(rightK) > K_MAX_THRESHOLD)
                    {
                        imageLine.Exist_Right[j] = 0;
                        continue;
                        //imageLine.Exist_Right[j] = 0;
                    }
                    break;        //只要找到一个临近有效点，检测后就break到下一个i
                }
                else
                    continue;
            }
        }
        else
            continue;
    }
}
/*丢边线*/
void lostLine_Filter(void)
{
    //对于左边界线的判断--------------------
    int count = 0;
    int i = 0;

    for (i = 0; i < ImageH; i++)        //从上到下搜索
    {
        if (imageLine.Exist_Left[i] == 1)
            count++;
    }

    if (count < VALID_LINE_THRESHOLE)        //如果无效行超过阈值认为该边界线丢失
    {
        imageLine.Lost_Left = 1;
        for (i = 0; i < ImageH; i++)        //从上到下搜索
        {
            imageLine.Exist_Left[i] = 0;
        }
    }
    else
        imageLine.Lost_Left = 0;

    //对于右边界线的判断--------------------
    count = 0;
    for (i = 0; i < ImageH; i++)
    {
        if (imageLine.Exist_Right[i] == 1)
            count++;
    }

    if (count < VALID_LINE_THRESHOLE)        //如果无效行超过阈值认为该边界线丢失
    {
        imageLine.Lost_Right = 1;
        for (i = 0; i < ImageH; i++)        //从上到下搜索
        {
            imageLine.Exist_Right[i] = 0;
        }
    }
    else
        imageLine.Lost_Right = 0;
}
/*滤除位置不对的点*/
void position_Filter(void)
{
    int i;

    //仅当左右都不丢线的时候才滤
    if (!imageLine.Lost_Left && !imageLine.Lost_Right)
    {
        for (i = 0; i < ImageH; i++)
        {
            if (imageLine.Exist_Left[i] && (imageLine.Point_Left[i] > ImageW) / 2)
                imageLine.Exist_Left[i] = 0;
            if (imageLine.Exist_Right[i] && (imageLine.Point_Right[i] < ImageW) / 2)
                imageLine.Exist_Right[i] = 0;
        }
    }
}
/*滤除斜率不对的边界点*/
void slope_Filter(void)
{
    short i, j;
    float leftK = 0;
    float rightK = 0;

    //左边界相邻两有效点斜率检测
    for (i = ImageH - 2; i > 0; i--)        //从下往上
    {
        if (imageLine.Exist_Left[i])        //先找到第一个有效点
        {
            for (j = i + 1; j < ImageH; j++)        //再向下找临近有效点
            {
                if (imageLine.Exist_Left[j])
                {
                    leftK = getLineK(i, imageLine.Point_Left[i], j, imageLine.Point_Left[j]);

                    if (leftK > 2 || abs(leftK) > K_MAX_THRESHOLD)        //左边线理想状态是一直往右的(xzl: 或许会影响到找拐点？)
                    {
                        //imageLine.Exist_Left[i] = 0;
                        imageLine.Exist_Left[j] = 0;
                    }
                    break;        //只要找到一个临近有效点，检测后就break到下一个i
                }
                else
                    continue;
            }
        }
        else
            continue;
    }

    //右边界相邻两有效点斜率检测
    for (i = ImageH - 2; i > 0; i--)        //从下往上
    {
        if (imageLine.Exist_Right[i])        //先找到第一个有效点
        {
            for (j = i + 1; j < ImageH; j++)        //再向下找临近有效点
            {
                if (imageLine.Exist_Right[j])
                {
                    rightK = getLineK(i, imageLine.Point_Right[i], j, imageLine.Point_Right[j]);

                    if (rightK < -2 || abs(rightK) > K_MAX_THRESHOLD)        //左边线理想状态是一直往右的(xzl: 或许会影响到找拐点？)
                    {
                        //imageLine.Exist_Right[i] = 0;
                        imageLine.Exist_Right[j] = 0;
                    }
                    break;        //只要找到一个临近有效点，检测后就break到下一个i
                }
                else
                    continue;
            }
        }
        else
            continue;
    }
}
/*滤除单个边界点*/
void singlePoint_Filter(void)
{
    int i;
    for (i = EFFECTIVE_ROW; i < ImageH - 1; i++)
    {
        if (!imageLine.Exist_Left[i - 1] && imageLine.Exist_Left[i] && !imageLine.Exist_Left[i + 1])
        {
            imageLine.Exist_Left[i] = 0;
        }
        if (!imageLine.Exist_Right[i - 1] && imageLine.Exist_Right[i] && !imageLine.Exist_Right[i + 1])
        {
            imageLine.Exist_Right[i] = 0;
        }
    }
    for (i = EFFECTIVE_ROW; i < ImageH - 2; i++)
    {
        if (!imageLine.Exist_Left[i - 1] && imageLine.Exist_Left[i] && imageLine.Exist_Left[i + 1]
            && !imageLine.Exist_Left[i + 2])
        {
            imageLine.Exist_Left[i] = 0;
            imageLine.Exist_Left[i + 1] = 0;
        }
        if (!imageLine.Exist_Right[i - 1] && imageLine.Exist_Right[i] && imageLine.Exist_Right[i + 1]
            && !imageLine.Exist_Right[i + 2])
        {
            imageLine.Exist_Right[i] = 0;
            imageLine.Exist_Right[i + 1] = 0;
        }
    }

}
/*****************************************************补线************************************************************************/
/*补线合集*/
void doMend(void)
{
    StraightLineJudge();    //为后面的后补线做准备
    trackMend_startPart();    //补前端(距离车)

    if ((!isLeftLineStraight && !imageLine.Lost_Left && imageLine.Lost_Right)
        || (!isRightLineStraight && imageLine.Lost_Left && !imageLine.Lost_Right)
        || (!isLeftLineStraight && !isRightLineStraight && !imageLine.Lost_Left && !imageLine.Lost_Right))
        trackMend_endPart();    //补末端(距离车)

    trackMend_HalfWidth();    //丢边线半宽补，不丢线直接计算
}
//直线判断
void StraightLineJudge(void)
{
    float k1, b1, k2, b2;
    short JudgeBasis_left[2][60];
    short JudgeBasis_right[2][60];

    int count1 = 0, count2 = 0;
    float err1, err2;

    int i, j;

    int temp_leftLine_lost = 0;    //丢左线（临时标志
    int temp_rightLine_lost = 0;    //丢右线（临时标志

    //拟合左线---------------------
    for (i = EFFECTIVE_ROW; i < ImageH; i++)
    {
        if (imageLine.Exist_Left[i])
        {
            JudgeBasis_left[0][count1] = (signed short int) i;
            JudgeBasis_left[1][count1] = (signed short int) imageLine.Point_Left[i];
            count1++;
        }
    }

    if (count1 >= VALID_LINE_THRESHOLE)
        leastSquareMethod(JudgeBasis_left[0], JudgeBasis_left[1], count1, &k1, &b1);
    else
        imageLine.Lost_Left = 1;

    //拟合右线---------------------
    for (i = EFFECTIVE_ROW; i < ImageH; i++)
    {
        if (imageLine.Exist_Right[i])
        {
            JudgeBasis_right[0][count2] = (signed short int) i;
            JudgeBasis_right[1][count2] = (signed short int) imageLine.Point_Right[i];
            count2++;
        }
    }

    if (count2 >= VALID_LINE_THRESHOLE)
        leastSquareMethod(JudgeBasis_right[0], JudgeBasis_right[1], count2, &k2, &b2);
    else
        imageLine.Lost_Right = 1;

    //第一次拟合后判断丢边情况---先判断左边界丢了没有 要是丢了就重新拟合；再同样判断右边界
    //如果左右边线都没有丢，就不进行下面的二次拟合
    //*************************************************
    //1. 只丢了左边线
    if (imageLine.Lost_Left && !imageLine.Lost_Right)
    {
        temp_leftLine_lost = 1;
        //左线丢了 再找一次左线----------
        for (i = EFFECTIVE_ROW; i < ImageH; i++)    //从上往下
        {
            if (imageLine.Exist_Right[i])    //找到右边界点
            {
                for (j = imageLine.Exist_Right[i] - 1; j > 0; j--)    //从右边界点这一行往左边找是否有左边界点
                {
                    if (isLeftPoint(i, j))
                    {
                        imageLine.Exist_Left[i] = 1;
                        imageLine.Point_Left[i] = j;
                        break;    //只要找到一个左边界点就不找了，认为左线没有丢
                    }
                }
            }
        }
        imageLine.Lost_Left = 0;

        //第二次拟合------
        count1 = 0;
        for (i = EFFECTIVE_ROW; i < ImageH; i++)
        {
            if (imageLine.Exist_Left[i])
            {
                JudgeBasis_left[0][count1] = (signed short int) i;
                JudgeBasis_left[1][count1] = (signed short int) imageLine.Point_Left[i];
                count1++;
            }
        }

        if (count1 >= VALID_LINE_THRESHOLE)
            leastSquareMethod(JudgeBasis_left[0], JudgeBasis_left[1], count1, &k1, &b1);
        else
            imageLine.Lost_Left = 1;
    }
    //2. 只丢了右边线
    else if (!imageLine.Lost_Left && imageLine.Lost_Right)
    {
        temp_rightLine_lost = 1;
        //右线丢了 再找一次右线----------
        for (i = EFFECTIVE_ROW; i < ImageH; i++)    //从上往下
        {
            if (imageLine.Exist_Left[i])    //找到左边界点
            {
                for (j = imageLine.Exist_Left[i] + 1; j < ImageW; j++)    //在左边界同一行开始找右边界点
                {
                    if (isRightPoint(i, j))
                    {
                        imageLine.Exist_Right[i] = 1;
                        imageLine.Point_Right[i] = j;
                        break;    //只要找到一个右边界点就不找了，认为右线没有丢
                    }
                }
            }
        }
        imageLine.Lost_Right = 0;

        //第二次拟合------
        count2 = 0;
        for (i = EFFECTIVE_ROW; i < ImageH; i++)
        {
            if (imageLine.Exist_Right[i])
            {
                JudgeBasis_right[0][count2] = (signed short int) i;
                JudgeBasis_right[1][count2] = (signed short int) imageLine.Point_Right[i];
                count2++;
            }
        }

        if (count2 >= VALID_LINE_THRESHOLE)
            leastSquareMethod(JudgeBasis_right[0], JudgeBasis_right[1], count2, &k2, &b2);
        else
            imageLine.Lost_Right = 1;
    }
    //*************************************************

    //直线（拟合误差）判断-------
    if (!imageLine.Lost_Left)    //左线没丢的情况下计算左拟合误差
    {
        err1 = getLeastSquareMethodERROR(JudgeBasis_left[0], JudgeBasis_left[1], count1, k1, b1);
        if (err1 > 1)
        {
            isLeftLineStraight = 0;    //左线是否为直线
        }
    }

    if (!imageLine.Lost_Right)    //右线没丢的情况下计算右拟合误差
    {
        err2 = getLeastSquareMethodERROR(JudgeBasis_right[0], JudgeBasis_right[1], count2, k2, b2);
        if (err2 > 1)
        {
            isRightLineStraight = 0;    //右线是否为直线
        }
    }

    if (imageLine.Lost_Left && imageLine.Lost_Right)    //如果左右边线都丢了
    {
        isLeftRightLineExist = 0;
    }
    //丢边线之后边界点都不要了-----------------------
    if (temp_leftLine_lost)
    {
        for (i = 0; i < ImageH; i++)
        {
            imageLine.Exist_Left[i] = 0;
        }
        imageLine.Lost_Left = 1;
    }
    if (temp_rightLine_lost)
    {
        for (i = 0; i < ImageH; i++)
        {
            imageLine.Exist_Right[i] = 0;
        }
        imageLine.Lost_Right = 1;
    }
}
/*补中线*/
void trackMend_startPart(void)
{
    int leftLine_startPoint = 0;
    int rightLine_startPoint = 0;
    int i;

    float k_left, b_left;
    float k_right, b_right;

    short MendBasis_left[2][5];
    short MendBasis_right[2][5];
    int count = 0;

    //------------------------补左线-------------------------------
    //找左线起始点
    for (i = ImageH - 1; i >= (ImageH / 2); i--)
    {
        if (imageLine.Exist_Left[i])
        {
            leftLine_startPoint = i;
            break;
        }
    }
    //------------------------补右线-------------------------------
    //找右线起始点
    for (i = ImageH - 1; i >= (ImageH / 2); i--)
    {
        if (imageLine.Exist_Right[i])
        {
            rightLine_startPoint = i;
            break;
        }
    }
    if (imageLine.Point_Right[rightLine_startPoint] - imageLine.Point_Left[leftLine_startPoint] >= 5)    //防止在三叉的位置补下部分线
    {
        //当左起始超过2/3行时(图像底部看不到边线的部分有点多啦),补线
        if (leftLine_startPoint > 30)
        {
            for (i = leftLine_startPoint; i >= (leftLine_startPoint - 15); i--)
            {
                if (imageLine.Exist_Left[i])
                {
                    MendBasis_left[0][count] = (short)i;
                    MendBasis_left[1][count] = (short)imageLine.Point_Left[i];
                    count++;
                }
                if (count == 5)
                    break;
            }
            if (count == 5)    //有5个点即可开始补线
            {
                leastSquareMethod(MendBasis_left[0], MendBasis_left[1], 5, &k_left, &b_left);

                //开始补线
                for (i = ImageH - 1; i >= leftLine_startPoint; i--)
                {
                    if (!imageLine.Exist_Left[i])
                    {
                        imageLine.Point_Left[i] = getLineValue(i, k_left, b_left);
                        imageLine.Exist_Left[i] = 1;
                    }

                }
            }
        }
        //当右起始超过2/3行时(图像底部看不到边线的部分有点多啦),补线
        count = 0;
        if (rightLine_startPoint > 30)
        {
            for (i = rightLine_startPoint; i >= rightLine_startPoint - 15; i--)
            {
                if (imageLine.Exist_Right[i])
                {
                    MendBasis_right[0][count] = (short)i;
                    MendBasis_right[1][count] = (short)imageLine.Point_Right[i];
                    count++;
                }
                if (count == 5)
                    break;
            }
            if (count == 5)    //有5个点即可开始补线
            {
                leastSquareMethod(MendBasis_right[0], MendBasis_right[1], 5, &k_right, &b_right);

                //开始补线
                for (i = ImageH - 1; i >= rightLine_startPoint; i--)
                {
                    if (!imageLine.Exist_Right[i])
                    {
                        imageLine.Point_Right[i] = getLineValue(i, k_right, b_right);
                        imageLine.Exist_Right[i] = 1;
                    }
                }
            }
        }
    }
}
void trackMend_HalfWidth(void)
{
    int i, j;
    float err = 0, aveErr = 0;
    int count = 0;
    int centerCompensation = 0;
    float aveerrmax = 0;
    int Start_Line = 0;
    //(一) 两边丢线(没救了)------------------------------------
    if (imageLine.Lost_Left && imageLine.Lost_Right)
    {
        imageLine.Lost_Center = 1;
    }
    //(二) 只丢左边线
    else if (imageLine.Lost_Left)
    {
        imageLine.Lost_Center = 0;

        //1. 计算中线补偿
        for (i = ImageH - 1; i >= 1; i--)
        {
            if (imageLine.Exist_Right[i])
            {
                err += (((float)ImageW / 2 + roadK * i / 2 + roadB / 2) - imageLine.Point_Right[i]);
                count++;
            }
        }
        //2. 计算平均误差
        aveErr = (float)(err / count);

        aveerrmax = MAX(aveErr, aveerrmax);    //用于调参

        //3. 根据右边线补中线(路宽+中线补偿)
        if (count >= 5 && aveErr > 0)    //点数足够且确定右边界线往左倾斜
            centerCompensation = LIMIT2MIN(aveErr, SingleLineLeanAveERR_MAX) / SingleLineLeanAveERR_MAX
            * SingleLineLeanK / 2;    //补偿计算

        for (i = ImageH - 1; i >= 1; i--)
        {
            if (imageLine.Exist_Right[i])
            {
                //计算中线并限幅
                imageLine.Exist_Center[i] = 1;
                amplitudeLIMIT(i, imageLine.Point_Right[i] - centerCompensation - roadK * i / 2 - roadB / 2 - (60 - i));
            }
        }
        limitCenter();    //对得出的中线进行突变限幅
        repairRemainLine();    //用最小二乘法修复未知的中线
    }
    //(三) 只丢右边线
    else if (imageLine.Lost_Right)
    {
        imageLine.Lost_Center = 0;

        //1. 计算中线补偿
        for (i = ImageH - 1; i >= 0; i--)
        {
            if (imageLine.Exist_Left[i])
            {
                err += (imageLine.Point_Left[i] - ((ImageW) / 2 - roadK * i / 2 - roadB / 2));
                count++;
            }
        }

        //2. 计算平均误差
        aveErr = (float)(err / count);
        aveerrmax = MAX(aveErr, aveerrmax);    //用于调参

        //3. 根据左边线补中线(路宽+中线补偿)
        if (count >= 5 && aveErr > 0)    //点数足够且确定右边界线往左倾斜
            centerCompensation = LIMIT2MIN(aveErr, SingleLineLeanAveERR_MAX) / SingleLineLeanAveERR_MAX
            * SingleLineLeanK / 2;    //补偿计算

        for (i = ImageH - 1; i >= 0; i--)
        {
            if (imageLine.Exist_Left[i])
            {
                //计算中线并限幅
                imageLine.Exist_Center[i] = 1;
                amplitudeLIMIT(i, imageLine.Point_Left[i] + centerCompensation + roadK * i / 2 + roadB / 2 + (60 - i));
            }
        }
        limitCenter();//对得出的中线进行突变限幅
        repairRemainLine();//用最小二乘法修复未知的中线
    }
    //(四)两边都没有丢线
    else if (!imageLine.Lost_Left && !imageLine.Lost_Right)
    {
        imageLine.Lost_Center = 0;

        for (i = ImageH - 1; i >= 1; i--)
        {
            if (imageLine.Exist_Left[i] && imageLine.Exist_Right[i])
            {
                if (imageLine.Point_Right[i] - imageLine.Point_Left[i] < MINRoadLen)
                    continue;
                else
                {
                    //计算中线并限幅
                    imageLine.Exist_Center[i] = 1;
                    amplitudeLIMIT(i, (imageLine.Point_Left[i] + imageLine.Point_Right[i]) / 2);
                }
                Start_Line = i;
            }
            else if ((!imageLine.Exist_Left[i] && imageLine.Exist_Right[i]))
            {
                for (j = Start_Line - 2; j > Start_Line - 20; j--)
                {
                    if (j > 2 && imageLine.Exist_Left[j])
                    {
                        Start_Line = 0;
                        break;
                    }
                }
                imageLine.Exist_Center[i] = 1;
                if (Start_Line > 30)
                    amplitudeLIMIT(i, (imageLine.Point_Right[i] - (roadK * i + roadB) / 2) - (Start_Line - i) * 1.5);
                else
                    amplitudeLIMIT(i, (imageLine.Point_Right[i] - (roadK * i + roadB) / 2));
            }
            else if ((imageLine.Exist_Left[i] && !imageLine.Exist_Right[i]))
            {
                for (j = Start_Line - 2; j > Start_Line - 20; j--)
                {

                    if (j > 2 && imageLine.Exist_Right[j])
                    {
                        Start_Line = 0;
                        break;
                    }
                }
                imageLine.Exist_Center[i] = 1;
                if (Start_Line > 30)
                    amplitudeLIMIT(i, (imageLine.Point_Left[i] + (roadK * i + roadB) / 2) + (Start_Line - i) * 1.5);
                else
                    amplitudeLIMIT(i, (imageLine.Point_Left[i] + (roadK * i + roadB) / 2));
            }
            else
                continue;
        }
        limitCenter();
        repairRemainLine();
    }
}
void trackMend_endPart(void)
{
    int leftIsAllRight = 1; //左线是否一直向右
    int rightIsAllLeft = 1; //右线是否一直向左

    int leftTopPoint; //左边界最高有效点
    int rightTopPoint; //有边界最高有效点

    int count = 0;

    int i, j; //永远的工具i工具j

    //九宫格补左线---------------------------------
    //1. 判断左线是否一直向右
    for (i = ImageH - 1; i >= 0; i--)
    {
        if (imageLine.Exist_Left[i])
        {
            for (j = i - 1; j >= 0; j--)
            {
                if (imageLine.Exist_Left[j])
                {
                    if (imageLine.Point_Left[j] - imageLine.Point_Left[i] < 0)
                    {
                        leftIsAllRight = 0;
                        break;    //下一个i
                    }
                }
            }
            count++;
            leftTopPoint = i;    //最上最右的左边界点
        }
    }
    //2. 左线满足一直往右，开始八点寻边
    if (leftIsAllRight && (count > 15))
    {
        int tempPointer1;
        int tempPointer_Val1;

        tempPointer1 = leftTopPoint;    //最上最右的左边界点
        tempPointer_Val1 = imageLine.Point_Left[tempPointer1];

        while (1)
        {
            //右上
            if (isEdgePoint(tempPointer1 - 1, tempPointer_Val1 + 1))
            {
                tempPointer1 = tempPointer1 - 1;
                tempPointer_Val1 = tempPointer_Val1 + 1;
                imageLine.Exist_Left[tempPointer1] = 1;
                imageLine.Point_Left[tempPointer1] = tempPointer_Val1;
            }
            //上
            else if (isEdgePoint(tempPointer1 - 1, tempPointer_Val1))
            {
                tempPointer1 = tempPointer1 - 1;
                imageLine.Exist_Left[tempPointer1] = 1;
                imageLine.Point_Left[tempPointer1] = tempPointer_Val1;
            }
            //右
            else if (isEdgePoint(tempPointer1, tempPointer_Val1 + 1))
            {
                tempPointer_Val1 = tempPointer_Val1 + 1;
                imageLine.Exist_Left[tempPointer1] = 1;
                imageLine.Point_Left[tempPointer1] = tempPointer_Val1;
            }
            else
                break;
        }
    }

    //九宫格补右线---------------------------------
    count = 0;
    //1. 判断右线是否一直向左
    for (i = ImageH - 1; i >= 0; i--)
    {
        if (imageLine.Exist_Right[i])
        {
            for (j = i + 1; j < ImageH; j++)
            {
                if (imageLine.Exist_Right[j])
                {
                    if (imageLine.Point_Right[j] - imageLine.Point_Right[i] < 0)
                    {
                        rightIsAllLeft = 0;
                        break;    //下一个i
                    }
                }
            }
            count++;
            rightTopPoint = i;    //最上最左的右边界点
        }
    }
    //2. 右线满足一直往左，开始八点寻边
    if (rightIsAllLeft && (count > 15))
    {
        int tempPointer2;
        int tempPointer_Val2;

        tempPointer2 = rightTopPoint;    //最上最左的右边界点
        tempPointer_Val2 = imageLine.Point_Right[tempPointer2];

        while (1)
        {
            //左上
            if (isEdgePoint(tempPointer2 - 1, tempPointer_Val2 - 1))
            {
                tempPointer2 = tempPointer2 - 1;
                tempPointer_Val2 = tempPointer_Val2 - 1;
                imageLine.Exist_Right[tempPointer2] = 1;
                imageLine.Point_Right[tempPointer2] = tempPointer_Val2;
            }
            //上
            else if (isEdgePoint(tempPointer2 - 1, tempPointer_Val2))
            {
                tempPointer2 = tempPointer2 - 1;
                imageLine.Exist_Right[tempPointer2] = 1;
                imageLine.Point_Right[tempPointer2] = tempPointer_Val2;
            }
            //右
            else if (isEdgePoint(tempPointer2, tempPointer_Val2 - 1))
            {

                tempPointer_Val2 = tempPointer_Val2 - 1;
                imageLine.Exist_Right[tempPointer2] = 1;
                imageLine.Point_Right[tempPointer2] = tempPointer_Val2;
            }
            else
                break;
        }
    }
}
/*仅用于九宫格搜索(在已搜索到边界点的条件下，找下个边界点的条件)*/
int isEdgePoint(int i, int j)
{
    if (j < 2 || j >= ImageW - 3 || i < 0 || i > ImageH - 1)        //图像边缘
        return 0;
    else if (((isWhite(i, j)))        //本身是白色
        && ((!isWhite(i + 1, j)) || (!isWhite(i - 1, j)) || (!isWhite(i, j + 1)) || (!isWhite(i, j - 1)))      //上下左右至少有一个黑色
        )
        return 1;
    else
        return 0;
}
/*最小二乘法拟合直线
参数：x：x坐标数组
y:y坐标数组
len:数据长度
k：返回直线k 值
b: 返回直线b 值
确保x和y数组大小一样！！！！！！！*/
void leastSquareMethod(short* x, short* y, int len, float* k, float* b)
{
    int i;

    int sumXY, sumX, sumY, sumX2 = 0;
    float aveX, aveY;

    sumXY = sumX = sumY = sumX2 = 0;

    for (i = 0; i < len; i++)
    {
        sumXY += x[i] * y[i];
        sumX += x[i];
        sumY += y[i];
        sumX2 += x[i] * x[i];
    }

    aveX = (float)sumX / (float)len;
    aveY = (float)sumY / (float)len;

    *k = (float)(sumXY - sumX * sumY / len) / (sumX2 - sumX * sumX / len);
    *b = (float)(aveY - (*k) * aveX);
}
/*用最小二乘法修复未知的中线。这里的前后是相对于摄像头的距离来说的，而不是图像，也就是说，图像的下方被称为前(起始)*/
void repairRemainLine(void)
{
    int i, j;
    int mediumLineStart = 0, mediumLineEnd = 0;

    int x1, y1;
    int x2 = 0, y2;

    short mediumLine1[2][60], mediumLine2[2][60];        //起点到拐点1的线段，拐点1到拐点2的线段

    int count1, count2;

    float k1, k2, b1, b2;

    //(一) 拟合前面部分
    //1. 先找到最下面的第一个有效中点-----------------------
    for (i = ImageH - 1; i >= 0; i--)    //从下往上
    {
        if (imageLine.Exist_Center[i])
        {
            mediumLineStart = i;
            break;
        }
    }

    //2. 填补残缺点----------------------------------------
    for (i = mediumLineStart - 1; i >= 0; i--)
    {
        if (!imageLine.Exist_Center[i])    //有效的起始中点的上一行中点没有
        {
            x1 = i + 1;
            y1 = imageLine.Point_Center[x1];    //起始中点的下一行点

            for (j = i - 1; j >= 0; j--)    //往上继续找有效中点
            {
                if (imageLine.Exist_Center[j])
                {
                    x2 = j;
                    y2 = imageLine.Point_Center[x2];
                    for (j = x2; j <= x1; j++)
                    {
                        imageLine.Exist_Center[j] = 1;
                        imageLine.Point_Center[j] = (short)((y2 - y1) * (j - x1) / (x2 - x1) + y1);
                    }
                    break;
                }
            }

        }
        else
            continue;
    }

    //3. 拟合图像下方的中线-----------------------------------
    count1 = 0;
    if (mediumLineStart >= 5)
    {
        for (i = mediumLineStart; i >= mediumLineStart - 5; i--)
            if (imageLine.Exist_Center[i])
            {
                mediumLine1[0][count1] = (short)i;
                mediumLine1[1][count1] = (short)imageLine.Point_Center[i];
                count1++;
            }

        if (count1 > 1)
        {
            leastSquareMethod(mediumLine1[0], mediumLine1[1], count1, &k1, &b1);
            for (i = ImageH - 1; i > mediumLineStart; i--)
            {
                //int8_t temp = (int8_t)(k1*i + b1);
                imageLine.Point_Center[i] = (k1 * i + b1);
                imageLine.Exist_Center[i] = 1;
            }
        }
    }

    //(二)拟合后面部分-----------------------------------
    //1. 先找到图像最上面的有效中点(中线终止点)
    for (i = 0; i < ImageH; i++)
    {
        if (imageLine.Exist_Center[i])
        {
            mediumLineEnd = i;
            break;
        }
    }
    //2. 拟合图像上方的中线
    count2 = 0;
    if (mediumLineEnd < ImageH - 5)
    {
        for (i = mediumLineEnd; i <= mediumLineEnd + 5; i++)
            if (imageLine.Exist_Center[i])
            {
                mediumLine2[0][count2] = (short)i;
                mediumLine2[1][count2] = (short)imageLine.Point_Center[i];
                count2++;
            }

        if (count2 > 1)
        {
            leastSquareMethod(mediumLine2[0], mediumLine2[1], count2, &k2, &b2);
            for (i = 0; i < mediumLineEnd; i++)
            {
                //int8_t temp = (int8_t)(k2*i + b2);
                imageLine.Point_Center[i] = (k2 * i + b2);
                imageLine.Exist_Center[i] = 1;
            }
        }
    }
}
float getLeastSquareMethodERROR(short* x, short* y, int len, float k, float b)
{
    float fitVal = 0;
    int i;
    float fitErr = 0;

    for (i = 0; i < len; i++)
    {
        fitVal = x[i] * k + b;
        fitErr += (fitVal - y[i]) * (fitVal - y[i]);
    }

    return (float)(fitErr / len);
}
/*对得出的中线进行赋值限幅（其实并没有这个功能） 给中点赋值，并认为该行中线有效*/
void amplitudeLIMIT(int i, int amp)
{
    imageLine.Exist_Center[i] = 1;
    imageLine.Point_Center[i] = amp;
}
/*对得出的中线进行突变限幅*/
void limitCenter(void)
{
    for (int i = ImageH - 2; i >= 1; i--)        //从下往上
    {
        if (imageLine.Exist_Center[i] && imageLine.Exist_Center[i + 1])
        {
            if (my_abs_short(imageLine.Point_Center[i] - imageLine.Point_Center[i + 1]) > 6)
            {
                if ((imageLine.Exist_Center[i]) && (!imageLine.Exist_Center[i - 1]))
                    imageLine.Exist_Center[i] = 0;
            }
        }
        if (imageLine.Exist_Center[i] && imageLine.Exist_Center[i - 1])
        {
            if (my_abs_short(imageLine.Point_Center[i] - imageLine.Point_Center[i - 1]) > 6)
            {
                if ((imageLine.Exist_Center[i]) && (!imageLine.Exist_Center[i + 1]))
                    imageLine.Exist_Center[i] = 0;
            }
        }
        if (imageLine.Exist_Center[i] && !imageLine.Exist_Center[i - 1] && !imageLine.Exist_Center[i + 1])
        {
            imageLine.Exist_Center[i] = 0;
        }
    }
}
/********************************************************三叉**********************************************************/
void ForkMend(void)
{
    if (L_Ring || R_Ring || T_Left_Road || T_Right_Road)
    {
        return;
    }

    if (ForkRoad_Cnt == 0)
    {
        OutsideFork_Mend();
        InsideFork_Mend();
        TiltFork_FliterR();
        ForkRoad_States();
    }
    if (ForkRoad_Cnt == 1 && Flag_Left_Ring_Clc == 1)
    {
        OutsideFork_Mend();
        InsideFork_Mend();
        TiltFork_FliterL();
        ForkRoad_States();
    }
}

void TiltFork_FliterL(void)
{
    int i = 0, j = 0, k = 0;
    int R_cnt = 0;
    int turn_Point_Row = EFFECTIVE_ROW;
    short flag_L_increase1 = 0, flag_L_decrease2 = 0, flag_L_increase3 = 0;
    static short increase_count1 = 0, increase_count2 = 0, decrease_count = 0;

    if (ForkRoad)
        return;
    /************************左边界点先向右再向左再向右***********************/
    for (i = ImageH - 1; i > EFFECTIVE_ROW; i--)
    {
        if ((imageLine.Point_Left[i] - imageLine.Point_Left[i - 1] < 0) && !flag_L_increase1)
        {
            increase_count1++;
            decrease_count = 0;
            if (increase_count1 >= 5)
            {
                increase_count1 = 0;
                flag_L_increase1 = 1;
                flag_L_decrease2 = 0;
            }
        }
        if ((imageLine.Point_Left[i] - imageLine.Point_Left[i - 1] > 0) && flag_L_increase1)
        {
            decrease_count++;
            increase_count1 = 0;
            if (decrease_count >= 5)
            {
                decrease_count = 0;
                flag_L_decrease2 = 1;
            }
        }
        if ((imageLine.Point_Left[i] - imageLine.Point_Left[i - 1] < 0) && flag_L_decrease2)
        {
            increase_count2++;
            decrease_count = 0;
            if (increase_count2 >= 2)
            {
                flag_L_increase3 = 1;
                break;
            }
        }
    }
    for (i = ImageH - 5; i > EFFECTIVE_ROW; i--)
    {
        if (imageLine.Exist_Right[i] && imageLine.Exist_Right[i - 1] && imageLine.Exist_Right[i - 2])
        {
            for (j = i; j > EFFECTIVE_ROW; j--)
            {
                if (imageLine.Exist_Right[j])
                {
                    R_cnt++;
                }
                if (!imageLine.Exist_Right[j] && !imageLine.Exist_Right[j - 1] && !imageLine.Exist_Right[j - 2] &&
                    !imageLine.Exist_Right[j - 3] && !imageLine.Exist_Right[j - 4])
                    break;
            }
        }
        if (!imageLine.Exist_Right[j] && !imageLine.Exist_Right[j - 1] && !imageLine.Exist_Right[j - 2])
            break;
    }
    if (R_cnt >= 40 || !flag_L_increase3)
        return;
    /************************找左边界拐点***********************/
    for (i = ImageH - 10; i > ImageH / 5; i--)
    {
        if (imageLine.Exist_Left[i])
        {
            for (j = i + 1; j <= ImageH - 10; j++)
            {
                if (imageLine.Exist_Left[j] && imageLine.Point_Left[j] - imageLine.Point_Left[i] > 0)
                {
                    turn_Point_Row = j;
                    break;
                }
            }
            if (turn_Point_Row != EFFECTIVE_ROW)
            {
                for (k = j + 1; k <= ImageH - 10; k++)
                {
                    if (imageLine.Exist_Left[k] && imageLine.Point_Left[k] == imageLine.Point_Left[turn_Point_Row])
                    {
                        turn_Point_Row = k;
                    }
                }
            }
        }
        if (turn_Point_Row != EFFECTIVE_ROW)
            break;
    }
    if (turn_Point_Row == EFFECTIVE_ROW || imageLine.Point_Left[turn_Point_Row] <= 2)
        return;

    /************************斜入三叉的滤波补线***********************/
    //滤波
    for (i = turn_Point_Row - 1; i >= EFFECTIVE_ROW; i--)
    {
        imageLine.Exist_Left[i] = 0;
        imageLine.Exist_Right[i] = 0;
    }
    //补线
    float k_left = 0, b_left = 0;
    short MendBasis_left[2][7];
    int count = 0;
    for (i = (turn_Point_Row + 5); i <= (turn_Point_Row + 15); i++)
    {
        if (imageLine.Exist_Left[i])
        {
            MendBasis_left[0][count] = (short)i;
            MendBasis_left[1][count] = (short)imageLine.Point_Left[i];
            count++;
        }
        if (count == 7)
            break;
    }
    if (count == 7)    //有7个点即可开始补线
    {
        leastSquareMethod(MendBasis_left[0], MendBasis_left[1], 7, &k_left, &b_left);

        //开始补线
        for (i = turn_Point_Row - 1; i >= EFFECTIVE_ROW; i--)
        {
            imageLine.Point_Left[i] = getLineValue(i, k_left, b_left);
            imageLine.Exist_Left[i] = 1;
            if (imageLine.Point_Right[i] > imageLine.Point_Left[i])
            {
                imageLine.Exist_Right[i] = 0;
            }
        }
    }
}
void TiltFork_FliterR(void)
{
    int i = 0, j = 0, k = 0;
    int L_cnt = 0;
    int turn_Point_Row = EFFECTIVE_ROW;
    short flag_R_decrease1 = 0, flag_R_increase2 = 0, flag_R_decrease3 = 0;
    static short decrease_count1 = 0, decrease_count2 = 0, increase_count = 0;

    if (ForkRoad)
        return;

    /************************右边界点先向左再向右再向左***********************/
    for (i = ImageH - 1; i > EFFECTIVE_ROW; i--)
    {
        if ((imageLine.Point_Right[i] - imageLine.Point_Right[i - 1] > 0) && !flag_R_decrease1)
        {
            decrease_count1++;
            increase_count = 0;
            if (decrease_count1 >= 5)//1.首先满足先减小趋势
            {
                decrease_count1 = 0;
                flag_R_decrease1 = 1;
                flag_R_increase2 = 0;
            }
        }
        if ((imageLine.Point_Right[i] - imageLine.Point_Right[i - 1] < 0) && flag_R_decrease1)//2.再满足白点数增加趋势
        {
            increase_count++;
            decrease_count1 = 0;
            if (increase_count >= 5)
            {
                increase_count = 0;
                flag_R_increase2 = 1;
            }
        }
        if ((imageLine.Point_Right[i] - imageLine.Point_Right[i - 1] > 0) && flag_R_increase2)
        {
            decrease_count2++;
            increase_count = 0;
            if (decrease_count2 >= 2)//1.首先满足先减小趋势
            {
                flag_R_decrease3 = 1;
                break;
            }
        }
    }
    for (i = ImageH - 5; i > EFFECTIVE_ROW; i--)
    {
        if (imageLine.Exist_Left[i] && imageLine.Exist_Left[i - 1] && imageLine.Exist_Left[i - 2])
        {
            for (j = i; j > EFFECTIVE_ROW; j--)
            {
                if (imageLine.Exist_Left[j])
                {
                    L_cnt++;
                }
                if (!imageLine.Exist_Left[j] && !imageLine.Exist_Left[j - 1] && !imageLine.Exist_Left[j - 2] &&
                    !imageLine.Exist_Left[j - 3] && !imageLine.Exist_Left[j - 4])
                    break;
            }
        }
        if (!imageLine.Exist_Left[j] && !imageLine.Exist_Left[j - 1] && !imageLine.Exist_Left[j - 2])
            break;
    }
    if (L_cnt >= 40 || !flag_R_decrease3)
        return;
    /************************找右边界拐点***********************/
    for (i = ImageH - 10; i > ImageH / 5; i--)
    {
        if (imageLine.Exist_Right[i])
        {
            for (j = i + 1; j <= ImageH - 10; j++)
            {
                if (imageLine.Exist_Right[j] && imageLine.Point_Right[j] - imageLine.Point_Right[i] < 0)
                {
                    turn_Point_Row = j;
                    break;
                }
            }
            if (turn_Point_Row != EFFECTIVE_ROW)
            {
                for (k = j + 1; k <= ImageH - 10; k++)
                {
                    if (imageLine.Exist_Right[k] && imageLine.Point_Right[k] == imageLine.Point_Right[turn_Point_Row])
                    {
                        turn_Point_Row = k;
                    }
                }
            }
        }
        if (turn_Point_Row != EFFECTIVE_ROW)
            break;
    }
    if (turn_Point_Row == EFFECTIVE_ROW || imageLine.Point_Right[turn_Point_Row] >= ImageW - 2)
        return;

    /************************斜入三叉的滤波补线***********************/
    //滤波
    for (i = turn_Point_Row - 1; i >= EFFECTIVE_ROW; i--)
    {
        imageLine.Exist_Left[i] = 0;
        imageLine.Exist_Right[i] = 0;
    }
    //补线
    float k_right = 0, b_right = 0;
    short MendBasis_right[2][7];
    int count = 0;
    for (i = (turn_Point_Row + 5); i <= (turn_Point_Row + 15); i++)
    {
        if (imageLine.Exist_Right[i])
        {
            MendBasis_right[0][count] = (short)i;
            MendBasis_right[1][count] = (short)imageLine.Point_Right[i];
            count++;
        }
        if (count == 7)
            break;
    }
    if (count == 7)    //有7个点即可开始补线
    {
        leastSquareMethod(MendBasis_right[0], MendBasis_right[1], 7, &k_right, &b_right);

        //开始补线
        for (i = turn_Point_Row - 1; i >= EFFECTIVE_ROW; i--)
        {
            imageLine.Point_Right[i] = getLineValue(i, k_right, b_right);
            imageLine.Exist_Right[i] = 1;
            if (imageLine.Point_Left[i] > imageLine.Point_Right[i])
            {
                imageLine.Exist_Left[i] = 0;
            }
        }
    }
}
void OutsideFork_Mend(void)
{
    int i = 0, j = 0, k = 0;
    int Divide_Point_Row = EFFECTIVE_ROW, Down_Point_Row = EFFECTIVE_ROW, Startline_R = EFFECTIVE_ROW;
    short flag_white_num_decrease1 = 0, flag_white_num_increase2 = 0, flag_white_num_decrease3 = 0;
    short decrease_count = 0, increase_count = 0, count = 0, flag = 0;
    int left_point[3] = { 0, 0, 0 }, right_point[3] = { 0, 0, 0 }, point_row[3] = { 0, 0, 0 };

    if (ForkRoad_Inside || ForkRoad_Out)
        return;

    /************************行白点数先减少再增加***********************/
    if (!Flag_ForkRoad_Mend)
    {
        for (i = ImageH - 10; i > ImageH / 6; i--)
        {
            if (imageLine.White_Num[i] - imageLine.White_Num[i - 1] > 0)
            {
                decrease_count++;
                increase_count = 0;
                if (decrease_count > 1)//1.首先满足先减小趋势
                {
                    decrease_count = 0;
                    flag_white_num_decrease1 = 1;
                    flag_white_num_increase2 = 0;
                }
                else if (flag_white_num_increase2 && imageLine.White_Num[i + 1] > 10)    //3.在满足先减小再增加趋势上满足再减小趋势 并为了跟一般的大弯区别开稍微修改条件
                {
                    flag_white_num_decrease3 = 1;
                }
            }
            else if ((imageLine.White_Num[i] - imageLine.White_Num[i - 1] < 0) && flag_white_num_decrease1)//2.再满足白点数增加趋势
            {
                decrease_count = 0;
                increase_count++;
                if (increase_count > 1)
                {
                    flag_white_num_increase2 = 1;
                }
            }
        }
        if (!flag_white_num_decrease3)
            return;
    }
    /************************找分界点***********************/
    for (i = ImageH - 1; i > ImageH / 6; i--)
    {
        if (imageLine.Exist_Right[i] && imageLine.Exist_Left[i] &&
            imageLine.Point_Right[i] < imageLine.Point_Left[i])
        {
            Divide_Point_Row = i;
            break;
        }
    }
    if (Divide_Point_Row == EFFECTIVE_ROW && !Flag_ForkRoad_Mend)
        return;
    /************************情况一补线***********************/
    if (Divide_Point_Row != EFFECTIVE_ROW)
    {
        /************************找顶点***********************/
        if (!Flag_ForkRoad_Mend)
        {
            for (i = Divide_Point_Row + 2; i >= Divide_Point_Row - 4; i--)
            {
                for (j = 1; j < ImageW - 1; j++)
                {
                    if (isWhite(i, j - 2) && isWhite(i, j - 1) && isWhite(i, j) && !isWhite(i, j + 1))
                    {
                        for (k = j; k < ImageW - 1; k++)
                        {
                            if (!isWhite(i, k - 1) && isWhite(i, k) && isWhite(i, k + 1) && isWhite(i, k + 2))
                            {
                                point_row[count] = i;
                                left_point[count] = j;
                                right_point[count] = k;
                                count++;
                                flag = 1;
                                break;
                            }
                        }
                    }
                    if (flag)
                    {
                        flag = 0;
                        break;
                    }
                }
                if (count == 3)
                    break;
            }
            if (count < 3)
                return;
        }
        else
        {
            for (i = Divide_Point_Row + 2; i >= Divide_Point_Row - 3; i--)
            {
                for (j = 3; j < ImageW - 1; j++)
                {
                    if (isWhite(i, j - 1) && isWhite(i, j) && !isWhite(i, j + 1))
                    {
                        for (k = j; k < ImageW - 1; k++)
                        {
                            if (!isWhite(i, k - 1) && isWhite(i, k) && isWhite(i, k + 1))
                            {
                                point_row[0] = i;
                                left_point[0] = j;
                                right_point[0] = k;
                                break;
                            }
                        }
                    }
                    if (point_row[0] == i)
                        break;
                }
                if (point_row[0] == i)
                    break;
            }
            if (point_row[0] == EFFECTIVE_ROW)
                return;
        }

        if (point_row[0] > 40 && left_point[0] < ImageW / 4)
        {
            return;
        }

        /************************找下拐点***********************/
        for (i = ImageH - 1; i > Divide_Point_Row; i--)
        {
            if ((i <= ImageH - 10) && (!imageLine.Exist_Right[i] && !imageLine.Exist_Right[i - 1] && !imageLine.Exist_Right[i - 2]
                && !imageLine.Exist_Right[i - 3] && !imageLine.Exist_Right[i - 4] && !imageLine.Exist_Right[i - 5]))
                break;
            if (imageLine.Exist_Right[i])
            {
                for (j = i + 1; j <= ImageH - 5; j++)
                {
                    if (imageLine.Exist_Right[j] && imageLine.Point_Right[j] - imageLine.Point_Right[i] < 0)
                    {
                        Down_Point_Row = j;
                        break;
                    }
                }
            }
            if (Down_Point_Row != EFFECTIVE_ROW)
                break;
            if (imageLine.Exist_Right[i] && !imageLine.Exist_Right[i - 1] && !imageLine.Exist_Right[i - 2] && !imageLine.Exist_Right[i - 3]
                && !imageLine.Exist_Right[i - 4] && !imageLine.Exist_Right[i - 5] && !imageLine.Exist_Right[i - 6])
            {
                Down_Point_Row = i;
                break;
            }
        }
        /************************滤波补线***********************/
        if (Down_Point_Row != EFFECTIVE_ROW)
        {
            //滤波
            for (i = 1; i < ImageH; i++)
            {
                if (imageLine.Point_Left[i] > left_point[0])
                    imageLine.Exist_Left[i] = 0;
            }
            for (i = 1; i < Down_Point_Row; i++)
            {
                if (imageLine.Point_Right[i] > left_point[0])
                    imageLine.Exist_Right[i] = 0;
            }
            //补线
            float k_right = 0, b_right = 0;
            k_right = ((float)left_point[0] - (float)imageLine.Point_Right[Down_Point_Row])
                / ((float)point_row[0] - (float)Down_Point_Row);
            b_right = (float)left_point[0] - k_right * point_row[0];
            for (i = point_row[0] + 1; i < Down_Point_Row; i++)
            {
                imageLine.Point_Right[i] = getLineValue(i, k_right, b_right);
                imageLine.Exist_Right[i] = 1;
                Flag_ForkRoad_Mend = 1;
            }
        }
        else
        {
            //滤波
            for (i = 1; i < ImageH; i++)
            {
                if (imageLine.Point_Right[i] > left_point[0])
                    imageLine.Exist_Right[i] = 0;
                if (imageLine.Point_Left[i] > left_point[0])
                    imageLine.Exist_Left[i] = 0;
            }
            //补线
            float k_right = 0, b_right = 0;
            k_right = ((float)left_point[0] - (float)89)
                / ((float)point_row[0] - (float)ImageH);
            b_right = (float)left_point[0] - k_right * point_row[0];
            for (i = point_row[0] + 1; i < ImageH; i++)
            {
                imageLine.Point_Right[i] = getLineValue(i, k_right, b_right);
                imageLine.Exist_Right[i] = 1;
                Flag_ForkRoad_Mend = 1;
            }
        }
    }
    /************************情况二补线***********************/
    if (Divide_Point_Row == EFFECTIVE_ROW && Flag_ForkRoad_Mend)
    {
        /************************找右边线起点***********************/
        for (i = ImageH - 5; i > ImageH / 6; i--)
        {
            if (imageLine.Exist_Right[i] && imageLine.Exist_Right[i - 1] && imageLine.Exist_Right[i - 2] &&
                (abs((imageLine.Point_Right[i - 1] + imageLine.Point_Right[i + 1]) / 2 - imageLine.Point_Right[i]) < 10))
            {
                Startline_R = i;
                break;
            }
        }
        /************************状态转换***********************/
        if (Startline_R >= 40)
        {
            Flag_ForkRoad_Mend = 0;
            Fork_Cnt++;
            return;
        }
        /************************滤波补线***********************/
        //滤波
        for (i = 1; i < Startline_R; i++)
        {
            if (imageLine.Point_Right[i] > imageLine.Point_Right[Startline_R])
                imageLine.Exist_Right[i] = 0;
            if (imageLine.Point_Left[i] > imageLine.Point_Right[Startline_R])
                imageLine.Exist_Left[i] = 0;
        }
        //补线
        float k_right = 0, b_right = 0;
        k_right = ((float)imageLine.Point_Right[Startline_R] - (float)89)
            / ((float)Startline_R - (float)ImageH);
        b_right = (float)imageLine.Point_Right[Startline_R] - k_right * Startline_R;
        for (i = Startline_R + 1; i < ImageH; i++)
        {
            imageLine.Point_Right[i] = getLineValue(i, k_right, b_right);
            imageLine.Exist_Right[i] = 1;
            Flag_ForkRoad_Mend = 1;
        }
    }
}
void InsideFork_Mend(void)
{
    int i = 0, j = 0, k = 0;
    int Divide_Point_Row = EFFECTIVE_ROW, Down_Point_Row = EFFECTIVE_ROW, Startline_R = EFFECTIVE_ROW;
    int left_point = 0, right_point = 0, point_row = 0;
    if (!ForkRoad_Inside && !ForkRoad_Out)
        return;
    /************************找分界点***********************/
    for (i = ImageH - 1; i > ImageH / 6; i--)
    {
        if (imageLine.Exist_Right[i] && imageLine.Exist_Left[i] &&
            imageLine.Point_Right[i] < imageLine.Point_Left[i])
        {
            Divide_Point_Row = i;
            break;
        }
    }
    if (Divide_Point_Row == EFFECTIVE_ROW && !Flag_ForkRoad_Mend)
        return;
    /************************情况一补线***********************/
    if (Divide_Point_Row != EFFECTIVE_ROW)
    {
        /************************找顶点***********************/
        for (i = Divide_Point_Row + 2; i >= Divide_Point_Row - 3; i--)
        {
            for (j = 3; j < ImageW - 1; j++)
            {
                if (isWhite(i, j - 1) && isWhite(i, j) && !isWhite(i, j + 1))
                {
                    for (k = j; k < ImageW - 1; k++)
                    {
                        if (!isWhite(i, k - 1) && isWhite(i, k) && isWhite(i, k + 1))
                        {
                            point_row = i;
                            left_point = j;
                            right_point = k;
                            break;
                        }
                    }
                }
                if (point_row == i)
                    break;
            }
            if (point_row == i)
                break;
        }
        if (point_row == EFFECTIVE_ROW)
            return;
        /************************找下拐点***********************/
        for (i = ImageH - 1; i > Divide_Point_Row; i--)
        {
            if ((i <= ImageH - 10) && (!imageLine.Exist_Right[i] && !imageLine.Exist_Right[i - 1] && !imageLine.Exist_Right[i - 2]
                && !imageLine.Exist_Right[i - 3] && !imageLine.Exist_Right[i - 4] && !imageLine.Exist_Right[i - 5]))
                break;
            if (imageLine.Exist_Right[i])
            {
                for (j = i + 1; j <= ImageH - 5; j++)
                {
                    if (imageLine.Exist_Right[j] && imageLine.Point_Right[j] - imageLine.Point_Right[i] < 0)
                    {
                        Down_Point_Row = j;
                        break;
                    }
                }
            }
            if (Down_Point_Row != EFFECTIVE_ROW)
                break;
            if (imageLine.Exist_Right[i] && !imageLine.Exist_Right[i - 1] && !imageLine.Exist_Right[i - 2] && !imageLine.Exist_Right[i - 3]
                && !imageLine.Exist_Right[i - 4] && !imageLine.Exist_Right[i - 5] && !imageLine.Exist_Right[i - 6])
            {
                Down_Point_Row = i;
                break;
            }
        }
        /************************滤波补线***********************/
        if (Down_Point_Row != EFFECTIVE_ROW)
        {
            //滤波
            for (i = 1; i < ImageH; i++)
            {
                if (imageLine.Point_Left[i] > left_point)
                    imageLine.Exist_Left[i] = 0;
            }
            for (i = 1; i < Down_Point_Row; i++)
            {
                if (imageLine.Point_Right[i] > left_point)
                    imageLine.Exist_Right[i] = 0;
            }
            //补线
            float k_right = 0, b_right = 0;
            k_right = ((float)left_point - (float)imageLine.Point_Right[Down_Point_Row])
                / ((float)point_row - (float)Down_Point_Row);
            b_right = (float)left_point - k_right * point_row;
            for (i = point_row + 1; i < Down_Point_Row; i++)
            {
                imageLine.Point_Right[i] = getLineValue(i, k_right, b_right);
                imageLine.Exist_Right[i] = 1;
                Flag_ForkRoad_Mend = 1;
            }
        }
        else
        {
            //滤波
            for (i = 1; i < ImageH; i++)
            {
                if (imageLine.Point_Right[i] > left_point)
                    imageLine.Exist_Right[i] = 0;
                if (imageLine.Point_Left[i] > left_point)
                    imageLine.Exist_Left[i] = 0;
            }
            //补线
            float k_right = 0, b_right = 0;
            k_right = ((float)left_point - (float)89)
                / ((float)point_row - (float)ImageH);
            b_right = (float)left_point - k_right * point_row;
            for (i = point_row + 1; i < ImageH; i++)
            {
                imageLine.Point_Right[i] = getLineValue(i, k_right, b_right);
                imageLine.Exist_Right[i] = 1;
                Flag_ForkRoad_Mend = 1;
            }
        }
    }
    /************************情况二补线***********************/
    if (Divide_Point_Row == EFFECTIVE_ROW && Flag_ForkRoad_Mend)
    {
        /************************找右边线起点***********************/
        for (i = ImageH - 5; i > ImageH / 6; i--)
        {
            if (imageLine.Exist_Right[i] && imageLine.Exist_Right[i - 1] && imageLine.Exist_Right[i - 2] &&
                (abs((imageLine.Point_Right[i - 1] + imageLine.Point_Right[i + 1]) / 2 - imageLine.Point_Right[i]) < 10))
            {
                Startline_R = i;
                break;
            }
        }
        /************************状态转换***********************/
        if (Startline_R >= 40)
        {
            Flag_ForkRoad_Mend = 0;
            Fork_Cnt++;
            return;
        }
        /************************滤波补线***********************/
        //滤波
        for (i = 1; i < Startline_R; i++)
        {
            if (imageLine.Point_Right[i] > imageLine.Point_Right[Startline_R])
                imageLine.Exist_Right[i] = 0;
            if (imageLine.Point_Left[i] > imageLine.Point_Right[Startline_R])
                imageLine.Exist_Left[i] = 0;
        }
        //补线
        float k_right = 0, b_right = 0;
        k_right = ((float)imageLine.Point_Right[Startline_R] - (float)89)
            / ((float)Startline_R - (float)ImageH);
        b_right = (float)imageLine.Point_Right[Startline_R] - k_right * Startline_R;
        for (i = Startline_R + 1; i < ImageH; i++)
        {
            imageLine.Point_Right[i] = getLineValue(i, k_right, b_right);
            imageLine.Exist_Right[i] = 1;
            Flag_ForkRoad_Mend = 1;
        }
    }

}
void ForkRoad_States(void)
{
    if (!Flag_ForkRoad_Mend && Fork_Cnt == 0)
    {
        ForkRoad_In = 0;
        ForkRoad_Inside = 0;
        ForkRoad_Out = 0;
    }

    if (Flag_ForkRoad_Mend && Fork_Cnt == 0)
        ForkRoad_In = 1;

    if (!Flag_ForkRoad_Mend && Fork_Cnt == 1)
    {
        ForkRoad_In = 0;
        ForkRoad_Inside = 1;
    }

    if (Flag_ForkRoad_Mend && Fork_Cnt == 1)
    {
        ForkRoad_Inside = 0;
        ForkRoad_Out = 1;
    }

    if (Fork_Cnt == 2)
    {
        ForkRoad_Out = 0;
        Fork_Cnt = 0;
        ForkRoad_Cnt++;
    }

    if (ForkRoad_In || ForkRoad_Inside || ForkRoad_Out)
        ForkRoad = 1;
    else
        ForkRoad = 0;
}
/*****************************************************十字************************************************************************/
void Cross_Filter(void)
{
    short alpha = CurrentServoDty - Ui_Servo_Mid;   //当前打角（以ServoDuty为单位）
    if (alpha > 20)
    {
        CrossFliter_Right();
    }
    if (alpha < -20)
    {
        CrossFliter_Left();
    }

    CrossMend_Right();
    CrossMend_Left();
}

void CrossMend_Left(void)
{
    int i, j;
    int exist_cnt1 = 0, exist_cnt2 = 0, n_exist_cnt = 0;
    int down_point_row = EFFECTIVE_ROW, up_point_row = EFFECTIVE_ROW;

    for (j = ImageH - 3; j > 2 * ImageH / 3; j--)
    {
        if (imageLine.Exist_Left[j])
        {
            exist_cnt1++;
            if (!imageLine.Exist_Left[j - 1] && !imageLine.Exist_Left[j - 2] && !imageLine.Exist_Left[j - 3]
                && imageLine.Point_Left[j] >= imageLine.Point_Left[j + 1])
            {
                down_point_row = j;
                break;
            }
        }

        if (down_point_row != EFFECTIVE_ROW && imageLine.Point_Left[down_point_row] > 2)
        {
            break;
        }
    }

    for (j = ImageH - 3; j > ImageH / 4; j--)
    {
        if (!imageLine.Exist_Left[j] && !imageLine.Exist_Left[j - 1] && !imageLine.Exist_Left[j - 2] && !imageLine.Exist_Left[j - 3] && !imageLine.Exist_Left[j - 5])
        {
            for (i = j; i > EFFECTIVE_ROW; i--)
            {
                if (!imageLine.Exist_Left[i])
                {
                    n_exist_cnt++;
                }
                if (imageLine.Exist_Left[i + 1] && imageLine.Exist_Left[i]
                    && (imageLine.Point_Left[i] - imageLine.Point_Left[i + 1] < 5) && (imageLine.Point_Left[i + 1] <= imageLine.Point_Left[i]))
                {
                    up_point_row = i;
                }
                if (up_point_row != EFFECTIVE_ROW && imageLine.Point_Left[up_point_row] > 2)
                {
                    break;
                }
            }
        }
        if (up_point_row != EFFECTIVE_ROW && imageLine.Point_Left[up_point_row] > 2)
        {
            break;
        }
    }

    if (up_point_row == EFFECTIVE_ROW || imageLine.Point_Left[up_point_row] <= 2 || n_exist_cnt > 40)
    {
        return;
    }

    for (i = up_point_row; i > EFFECTIVE_ROW; i--)
    {
        if (imageLine.Exist_Left[i + 1] && imageLine.Exist_Left[i]
            && (imageLine.Point_Left[i] - imageLine.Point_Left[i + 1] < 5) && (imageLine.Point_Left[i + 1] <= imageLine.Point_Left[i]))
        {
            exist_cnt2++;
        }
    }

    if (exist_cnt1 >= 5 && down_point_row != EFFECTIVE_ROW && imageLine.Point_Left[down_point_row] > 2)
    {


        for (i = down_point_row; i > EFFECTIVE_ROW; i--)
        {
            if (imageLine.Point_Left[i] < imageLine.Point_Left[up_point_row])
            {
                imageLine.Exist_Left[i] = 0;
            }
            if (imageLine.Point_Right[i] < imageLine.Point_Left[up_point_row])
            {
                imageLine.Exist_Right[i] = 0;
            }
        }

        float k_Left = 0, b_Left = 0;
        k_Left = ((float)imageLine.Point_Left[down_point_row] - (float)imageLine.Point_Left[up_point_row])
            / ((float)down_point_row - (float)up_point_row);
        b_Left = (float)imageLine.Point_Left[up_point_row] - k_Left * up_point_row;
        if (k_Left <= -1.4 || k_Left >= -0.5)
        {
            k_Left = ((float)5 - (float)imageLine.Point_Left[up_point_row])
                / ((float)59 - (float)up_point_row);
            b_Left = (float)imageLine.Point_Left[up_point_row] - k_Left * up_point_row;
        }
        for (i = down_point_row; i > up_point_row; i--)
        {
            imageLine.Exist_Left[i] = 1;
            imageLine.Point_Left[i] = k_Left * i + b_Left;
        }
    }
    else if (up_point_row < 50 && exist_cnt2 >= 3)
    {
        for (i = ImageH; i > up_point_row; i--)
        {
            imageLine.Exist_Left[i] = 0;
        }

        for (i = up_point_row; i > 1; i--)
        {
            if (imageLine.Point_Left[i] < imageLine.Point_Left[up_point_row])
            {
                imageLine.Exist_Left[i] = 0;
            }
            if (imageLine.Point_Right[i] < imageLine.Point_Left[up_point_row])
            {
                imageLine.Exist_Right[i] = 0;
            }
        }

        float k_left = 0, b_left = 0;
        short MendBasis_left[2][5];
        int count = 0;
        for (i = up_point_row; i >= (up_point_row - 10); i--)
        {
            if (imageLine.Exist_Left[i])
            {
                MendBasis_left[0][count] = (short)i;
                MendBasis_left[1][count] = (short)imageLine.Point_Left[i];
                count++;
            }
            if (count == 5)
                break;
        }
        if (count == 5)    //有5个点即可开始补线
        {
            leastSquareMethod(MendBasis_left[0], MendBasis_left[1], 5, &k_left, &b_left);
            if (k_left <= -1.4 || k_left >= -0.5)
            {
                k_left = ((float)5 - (float)imageLine.Point_Left[up_point_row])
                    / ((float)59 - (float)up_point_row);
                b_left = (float)imageLine.Point_Left[up_point_row] - k_left * up_point_row;
            }
            //开始补线
            for (i = ImageH - 1; i >= up_point_row; i--)
            {
                if (!imageLine.Exist_Left[i])
                {
                    imageLine.Point_Left[i] = getLineValue(i, k_left, b_left);
                    imageLine.Exist_Left[i] = 1;
                }
            }
        }
        else
        {
            k_left = ((float)5 - (float)imageLine.Point_Left[up_point_row])
                / ((float)59 - (float)up_point_row);
            b_left = (float)imageLine.Point_Left[up_point_row] - k_left * up_point_row;
            //开始补线
            for (i = ImageH - 1; i >= up_point_row; i--)
            {
                if (!imageLine.Exist_Left[i])
                {
                    imageLine.Point_Left[i] = getLineValue(i, k_left, b_left);
                    imageLine.Exist_Left[i] = 1;
                }
            }
        }
    }



}
void CrossMend_Right(void)
{
    int i, j;
    int exist_cnt1 = 0, exist_cnt2 = 0, n_exist_cnt = 0;
    int down_point_row = EFFECTIVE_ROW, up_point_row = EFFECTIVE_ROW;

    for (j = ImageH - 3; j > 2 * ImageH / 3; j--)
    {
        if (imageLine.Exist_Right[j])
        {
            exist_cnt1++;
            if (!imageLine.Exist_Right[j - 1] && !imageLine.Exist_Right[j - 2] && !imageLine.Exist_Right[j - 3]
                && imageLine.Point_Right[j] <= imageLine.Point_Right[j + 1])
            {
                down_point_row = j;
                break;
            }
        }

        if (down_point_row != EFFECTIVE_ROW && imageLine.Point_Right[down_point_row] < ImageW - 2)
        {
            break;
        }
    }

    for (j = ImageH - 3; j > ImageH / 4; j--)
    {
        if (!imageLine.Exist_Right[j] && !imageLine.Exist_Right[j - 1] && !imageLine.Exist_Right[j - 2] && !imageLine.Exist_Right[j - 3] && !imageLine.Exist_Right[j - 4])
        {
            for (i = j; i > EFFECTIVE_ROW; i--)
            {
                if (!imageLine.Exist_Right[i])
                {
                    n_exist_cnt++;
                }
                if (imageLine.Exist_Right[i + 1] && imageLine.Exist_Right[i]
                    && (imageLine.Point_Right[i + 1] - imageLine.Point_Right[i] < 5) && (imageLine.Point_Right[i + 1] >= imageLine.Point_Right[i]))
                {
                    up_point_row = i;
                }
                if (up_point_row != EFFECTIVE_ROW && imageLine.Point_Right[up_point_row] < ImageW - 2)
                {
                    break;
                }
            }
        }
        if (up_point_row != EFFECTIVE_ROW && imageLine.Point_Right[up_point_row] < ImageW - 2)
        {
            break;
        }
    }

    if (up_point_row == EFFECTIVE_ROW || imageLine.Point_Right[up_point_row] >= ImageW - 2 || n_exist_cnt > 40)
    {
        return;
    }

    for (i = up_point_row; i > EFFECTIVE_ROW; i--)
    {
        if (imageLine.Exist_Right[i + 1] && imageLine.Exist_Right[i]
            && (imageLine.Point_Right[i + 1] - imageLine.Point_Right[i] < 5) && (imageLine.Point_Right[i + 1] >= imageLine.Point_Right[i]))
        {
            exist_cnt2++;
        }
    }

    if (exist_cnt1 >= 5 && down_point_row != EFFECTIVE_ROW && imageLine.Point_Right[down_point_row] < ImageW - 2)
    {
        for (i = down_point_row; i > EFFECTIVE_ROW; i--)
        {
            if (imageLine.Point_Right[i] > imageLine.Point_Right[up_point_row])
            {
                imageLine.Exist_Right[i] = 0;
            }
            if (imageLine.Point_Left[i] > imageLine.Point_Right[up_point_row])
            {
                imageLine.Exist_Left[i] = 0;
            }
        }

        float k_Right = 0, b_Right = 0;
        k_Right = ((float)imageLine.Point_Right[down_point_row] - (float)imageLine.Point_Right[up_point_row])
            / ((float)down_point_row - (float)up_point_row);
        b_Right = (float)imageLine.Point_Right[up_point_row] - k_Right * up_point_row;
        if (k_Right >= 1.4 || k_Right <= 0.5)
        {
            k_Right = ((float)89 - (float)imageLine.Point_Right[up_point_row])
                / ((float)59 - (float)up_point_row);
            b_Right = (float)imageLine.Point_Right[up_point_row] - k_Right * up_point_row;
        }
        for (i = down_point_row; i > up_point_row; i--)
        {
            imageLine.Exist_Right[i] = 1;
            imageLine.Point_Right[i] = k_Right * i + b_Right;
        }
    }
    else if (up_point_row < 50 && exist_cnt2 >= 3)
    {
        for (i = ImageH; i > up_point_row; i--)
        {
            imageLine.Exist_Right[i] = 0;
        }

        for (i = up_point_row; i > 1; i--)
        {
            if (imageLine.Point_Right[i] > imageLine.Point_Right[up_point_row])
            {
                imageLine.Exist_Right[i] = 0;
            }
            if (imageLine.Point_Left[i] > imageLine.Point_Right[up_point_row])
            {
                imageLine.Exist_Left[i] = 0;
            }
        }

        float k_right = 0, b_right = 0;
        short MendBasis_right[2][5];
        int count = 0;
        for (i = up_point_row; i >= up_point_row - 10; i--)
        {
            if (imageLine.Exist_Right[i])
            {
                MendBasis_right[0][count] = (short)i;
                MendBasis_right[1][count] = (short)imageLine.Point_Right[i];
                count++;
            }
            if (count == 5)
                break;
        }
        if (count == 5)    //有5个点即可开始补线
        {
            leastSquareMethod(MendBasis_right[0], MendBasis_right[1], 5, &k_right, &b_right);
            if (k_right >= 1.4 || k_right <= 0.5)
            {
                k_right = ((float)89 - (float)imageLine.Point_Right[up_point_row])
                    / ((float)59 - (float)up_point_row);
                b_right = (float)imageLine.Point_Right[up_point_row] - k_right * up_point_row;
            }
            //开始补线
            for (i = ImageH - 1; i >= up_point_row; i--)
            {
                if (!imageLine.Exist_Right[i])
                {
                    imageLine.Point_Right[i] = getLineValue(i, k_right, b_right);
                    imageLine.Exist_Right[i] = 1;
                }
            }
        }
        else
        {
            k_right = ((float)89 - (float)imageLine.Point_Right[up_point_row])
                / ((float)59 - (float)up_point_row);
            b_right = (float)imageLine.Point_Right[up_point_row] - k_right * up_point_row;
            //开始补线
            for (i = ImageH - 1; i >= up_point_row; i--)
            {
                if (!imageLine.Exist_Right[i])
                {
                    imageLine.Point_Right[i] = getLineValue(i, k_right, b_right);
                    imageLine.Exist_Right[i] = 1;
                }
            }
        }
    }



}
void CrossFliter_Left(void)
{
    int i;
    int R_high_cnt = 0, R_low_cnt = 0, R_cnt = 0, L_cnt = 0;
    int turn_Point_Row = EFFECTIVE_ROW;
    int StartLine_R = ImageH, StartLine_L = ImageH;

    for (i = ImageH - 5; i >= 10; i--)
    {
        if (imageLine.Exist_Right[i] && StartLine_R == ImageH)
        {
            StartLine_R = i;
        }
        if (imageLine.Exist_Right[i])
        {
            R_cnt++;
            if ((imageLine.Point_Right[i] <= imageLine.Point_Right[i + 1]) && (imageLine.Point_Right[i + 1] - imageLine.Point_Right[i] < 10))
            {
                R_low_cnt++;
            }

            if ((imageLine.Point_Right[i] > imageLine.Point_Right[i + 1]) && (imageLine.Point_Right[i] - imageLine.Point_Right[i + 1] < 5))
            {
                turn_Point_Row = i;
            }
        }
        if (turn_Point_Row != EFFECTIVE_ROW && imageLine.Point_Right[turn_Point_Row] < ImageW - 2)
        {
            break;
        }
    }

    if ((StartLine_R < ImageH - 15) || R_cnt == R_low_cnt || turn_Point_Row == EFFECTIVE_ROW || imageLine.Point_Right[turn_Point_Row] >= ImageW - 8)
    {
        return;
    }

    for (i = turn_Point_Row; i >= 10; i--)
    {
        if (imageLine.Exist_Right[i])
        {
            if ((imageLine.Point_Right[i] > imageLine.Point_Right[i + 1]) && (imageLine.Point_Right[i] - imageLine.Point_Right[i + 1] < 5))
            {
                R_high_cnt++;
            }
        }
    }

    for (i = turn_Point_Row + 1; i >= EFFECTIVE_ROW; i--)
    {
        if (imageLine.Exist_Left[i] && StartLine_L == 0)
        {
            if (imageLine.Point_Left[i] > imageLine.Point_Right[turn_Point_Row] - 10)
            {
                StartLine_L = i;
            }
        }
    }

    for (i = ImageH - 5; i >= turn_Point_Row; i--)
    {
        if (imageLine.Exist_Left[i])
        {
            L_cnt++;
        }
    }

    if (turn_Point_Row > 40 || R_high_cnt < 5 || L_cnt > 2 * (60 - turn_Point_Row) / 3)
    {
        return;
    }

    for (i = turn_Point_Row - 1; i >= EFFECTIVE_ROW; i--)
    {
        imageLine.Exist_Right[i] = 0;
    }
    for (i = StartLine_L; i >= EFFECTIVE_ROW; i--)
    {
        imageLine.Exist_Left[i] = 0;
    }

    float k_right = 0, b_right = 0;
    short MendBasis_right[2][5];
    int count = 0;
    for (i = (turn_Point_Row + 5); i <= (turn_Point_Row + 15); i++)
    {
        if (imageLine.Exist_Right[i])
        {
            MendBasis_right[0][count] = (short)i;
            MendBasis_right[1][count] = (short)imageLine.Point_Right[i];
            count++;
        }
        if (count == 5)
            break;
    }
    if (count == 5)    //有5个点即可开始补线
    {
        leastSquareMethod(MendBasis_right[0], MendBasis_right[1], 5, &k_right, &b_right);

        //开始补线
        for (i = turn_Point_Row - 1; i >= EFFECTIVE_ROW; i--)
        {
            if (!imageLine.Exist_Right[i])
            {
                imageLine.Point_Right[i] = getLineValue(i, k_right, b_right);
                imageLine.Exist_Right[i] = 1;
            }
        }
    }
}
void CrossFliter_Right(void)
{
    int i;
    int L_high_cnt = 0, L_low_cnt = 0, L_cnt = 0, R_cnt = 0;
    int turn_Point_Row = EFFECTIVE_ROW;
    int StartLine_R = ImageH, StartLine_L = ImageH;

    for (i = ImageH - 5; i >= 10; i--)
    {
        if (imageLine.Exist_Left[i] && StartLine_L == ImageH)
        {
            StartLine_L = i;
        }
        if (imageLine.Exist_Left[i])
        {
            L_cnt++;
            if ((imageLine.Point_Left[i] >= imageLine.Point_Left[i + 1]) && (imageLine.Point_Left[i] - imageLine.Point_Left[i + 1] < 10))
            {
                L_high_cnt++;
            }

            if ((imageLine.Point_Left[i] < imageLine.Point_Left[i + 1]) && (imageLine.Point_Left[i + 1] - imageLine.Point_Left[i + 1] < 5))
            {
                turn_Point_Row = i;
            }
        }
        if (turn_Point_Row != EFFECTIVE_ROW && imageLine.Point_Left[turn_Point_Row] > 2)
        {
            break;
        }
    }

    if ((StartLine_L < ImageH - 15) || L_cnt == L_high_cnt || turn_Point_Row == EFFECTIVE_ROW || imageLine.Point_Left[turn_Point_Row] <= 8)
    {
        return;
    }

    for (i = turn_Point_Row; i >= 10; i--)
    {
        if (imageLine.Exist_Left[i])
        {
            if ((imageLine.Point_Left[i] < imageLine.Point_Left[i + 1]) && (imageLine.Point_Left[i] - imageLine.Point_Left[i + 1] < 5))
            {
                L_low_cnt++;
            }
        }
    }

    for (i = turn_Point_Row + 1; i >= EFFECTIVE_ROW; i--)
    {
        if (imageLine.Exist_Right[i] && StartLine_R == 0)
        {
            if (imageLine.Point_Right[i] < imageLine.Point_Left[turn_Point_Row] + 10)
            {
                StartLine_R = i;
            }
        }
    }

    for (i = ImageH - 5; i >= turn_Point_Row; i--)
    {
        if (imageLine.Exist_Right[i])
        {
            R_cnt++;
        }
    }

    if (turn_Point_Row > 40 || L_low_cnt < 5 || R_cnt > 2 * (60 - turn_Point_Row) / 3)
    {
        return;
    }

    for (i = turn_Point_Row - 1; i >= EFFECTIVE_ROW; i--)
    {
        imageLine.Exist_Left[i] = 0;
    }
    for (i = StartLine_R; i >= EFFECTIVE_ROW; i--)
    {
        imageLine.Exist_Right[i] = 0;
    }

    float k_left = 0, b_left = 0;
    short MendBasis_left[2][5];
    int count = 0;
    for (i = (turn_Point_Row + 5); i <= (turn_Point_Row + 15); i++)
    {
        if (imageLine.Exist_Left[i])
        {
            MendBasis_left[0][count] = (short)i;
            MendBasis_left[1][count] = (short)imageLine.Point_Left[i];
            count++;
        }
        if (count == 5)
            break;
    }
    if (count == 5)    //有5个点即可开始补线
    {
        leastSquareMethod(MendBasis_left[0], MendBasis_left[1], 5, &k_left, &b_left);

        //开始补线
        for (i = turn_Point_Row - 1; i >= EFFECTIVE_ROW; i--)
        {
            if (!imageLine.Exist_Left[i])
            {
                imageLine.Point_Left[i] = getLineValue(i, k_left, b_left);
                imageLine.Exist_Left[i] = 1;
            }
        }
    }
}
/*****************************************************圆环************************************************************************/
void Ring_Fliter(void)
{
    if (R_Ring || Flag_Right_Ring_Clc)
    {
        return;
    }

    int i = 0, j = 0;
    int R_low_cnt = 0, R_cnt = 0, R_Min = ImageW;
    int turn_Point_Row = EFFECTIVE_ROW;
    int StartLine_L = 0;

    for (i = ImageH - 10; i >= 10; i--)
    {
        if (!imageLine.Exist_Right[i] && !imageLine.Exist_Right[i - 1] && !imageLine.Exist_Right[i - 2]
            && !imageLine.Exist_Right[i - 3] && !imageLine.Exist_Right[i - 4] && !imageLine.Exist_Right[i - 5])
        {
            break;
        }
        if (imageLine.Exist_Right[i])
        {
            R_cnt++;
            if (imageLine.Point_Right[i] <= imageLine.Point_Right[i + 1])
            {
                R_low_cnt++;
            }

            if (imageLine.Point_Right[i] > imageLine.Point_Right[i + 1])
            {
                for (j = i; j <= ImageH - 10; j++)
                {
                    if (imageLine.Point_Right[j] <= R_Min)
                    {
                        turn_Point_Row = j;
                        R_Min = imageLine.Point_Right[j];
                    }
                }
            }
        }
        if (turn_Point_Row != EFFECTIVE_ROW)
        {
            break;
        }
    }

    if (R_cnt == R_low_cnt || turn_Point_Row > 45 || turn_Point_Row == EFFECTIVE_ROW || imageLine.Point_Right[turn_Point_Row] >= ImageW - 2)
    {
        return;
    }

    for (i = ImageH - 1; i >= EFFECTIVE_ROW; i--)
    {
        if (imageLine.Exist_Left[i] && imageLine.Point_Left[i] > 2)
        {
            StartLine_L = i;
            break;
        }
    }

    if (StartLine_L > 40)
    {
        //判断右边线斜率是否正常
        int count_real = 0;
        float k_real = 0, b_real = 0;
        short real_dot[2][10];
        for (i = 0; i < 2; i++)
        {
            for (j = 0; j < 10; j++)
            {
                real_dot[i][j] = 0;
            }
        }

        for (i = ImageH - 20; i >= EFFECTIVE_ROW; i--)
        {
            if (imageLine.Exist_Left[i])
            {
                real_dot[0][count_real] = (short)i;
                real_dot[1][count_real] = (short)imageLine.Point_Left[i];
                count_real++;
            }
            if (count_real == 10 || !imageLine.Exist_Left[i] || (imageLine.Exist_Left[i] && imageLine.Exist_Left[i - 1]
                && imageLine.Point_Left[i - 1] - imageLine.Point_Left[i] > 5))
                break;
        }
        if (count_real == 10)
        {
            leastSquareMethod(real_dot[0], real_dot[1], 10, &k_real, &b_real);
        }
        if (k_real < -0.6)
            return;
    }
    else if (StartLine_L > 30)
    {
        for (i = ImageH; i >= EFFECTIVE_ROW; i--)
        {
            imageLine.Exist_Left[i] = 1;
            imageLine.Point_Left[i] = 1;
        }
    }
    else
    {
        for (i = ImageH; i >= EFFECTIVE_ROW; i--)
        {
            imageLine.Exist_Left[i] = 0;
        }
    }

    for (i = turn_Point_Row - 1; i >= EFFECTIVE_ROW; i--)
    {
        imageLine.Exist_Right[i] = 0;
    }

    float k_right = 0, b_right = 0;
    short MendBasis_right[2][5];
    int count = 0;
    for (i = turn_Point_Row; i <= (turn_Point_Row + 15); i++)
    {
        if (imageLine.Exist_Right[i])
        {
            MendBasis_right[0][count] = (short)i;
            MendBasis_right[1][count] = (short)imageLine.Point_Right[i];
            count++;
        }
        if (count == 5)
            break;
    }
    if (count == 5)    //有5个点即可开始补线
    {
        leastSquareMethod(MendBasis_right[0], MendBasis_right[1], 5, &k_right, &b_right);

        //开始补线
        for (i = turn_Point_Row - 1; i >= EFFECTIVE_ROW; i--)
        {
            if (!imageLine.Exist_Right[i])
            {
                imageLine.Point_Right[i] = getLineValue(i, k_right, b_right);
                imageLine.Exist_Right[i] = 1;
            }
            if (imageLine.Point_Left[i] > imageLine.Point_Right[i])
            {
                imageLine.Exist_Left[i] = 0;
            }
        }
    }
}

void Left_Ring(void)
{
    if (Flag_T_Left_Clc && Flag_Left_Ring_OnlyOnce)
    {
        Left_Ring_Find();
        Left_Ring_Turn();
        Left_Ring_Out();
        Left_Ring_Out_Mend();
    }
    if (Flag_Left_Ring_Find || Flag_Left_Ring_Turn || Flag_Left_Ring_Out || Flag_Left_Ring_Out_Mend)
    {
        L_Ring = 1;
    }
    else
        L_Ring = 0;
}
void Left_Ring_Find(void)
{
    int i = 0, j = 0;
    int Down_Point = EFFECTIVE_ROW, Divide_Point = EFFECTIVE_ROW, Mid_Point = EFFECTIVE_ROW, Top_Point = EFFECTIVE_ROW;
    int L_Max = 0;

    if (Flag_Left_Ring_Turn || Flag_Left_Ring_Out || Flag_Left_Ring_Out_Mend)//在圆环的后状态则不执行找圆环
        return;
    /************************找环岛下直道端点***********************/
    for (i = ImageH - 5; i > ImageH / 3; i--)
    {
        if ((i <= ImageH - 10) && (!imageLine.Exist_Left[i] && !imageLine.Exist_Left[i - 1] && !imageLine.Exist_Left[i - 2]
            && !imageLine.Exist_Left[i - 3] && !imageLine.Exist_Left[i - 4] && !imageLine.Exist_Left[i - 5]))
            break;
        if (imageLine.Exist_Left[i])
        {
            for (j = i + 1; j <= ImageH - 5; j++)
            {
                if (imageLine.Exist_Left[j] && imageLine.Point_Left[j] - imageLine.Point_Left[i] > 0)
                {
                    Down_Point = j;
                    break;
                }
            }
        }
        if (Down_Point != EFFECTIVE_ROW)
        {
            break;
        }
        if (imageLine.Exist_Left[i] && !imageLine.Exist_Left[i - 1] && !imageLine.Exist_Left[i - 2] && !imageLine.Exist_Left[i - 3]
            && !imageLine.Exist_Left[i - 4] && !imageLine.Exist_Left[i - 5] && !imageLine.Exist_Left[i - 6])
        {
            Down_Point = i;
            break;
        }
    }
    if (Down_Point != EFFECTIVE_ROW)
    {
        int e_cnt = 0;
        for (i = Down_Point; i >= Down_Point - 10; i--)
        {
            if (imageLine.Exist_Left[i] && imageLine.Point_Left[Down_Point] - imageLine.Point_Left[i] < 10
                && imageLine.Point_Left[Down_Point] > imageLine.Point_Left[i])
            {
                e_cnt++;
            }
        }
        if (e_cnt >= 3)
            Down_Point = EFFECTIVE_ROW;
    }
    /************************找环岛下方直道分界点***********************/
    if (Down_Point != EFFECTIVE_ROW)
    {
        for (i = Down_Point - 3; i > ImageH / 5; i--)
        {
            if (imageLine.Exist_Left[i] && imageLine.Point_Left[i] > imageLine.Point_Left[Down_Point])
            {
                Divide_Point = i;
                break;
            }
        }
    }
    else
    {
        for (j = ImageH - 15; j > ImageH / 5; j--)
        {
            if (imageLine.Exist_Left[j] && imageLine.Point_Left[j] > 2)
            {
                Divide_Point = j;
                break;
            }
        }
    }
    /************************找环岛转折点***********************/
    if (Divide_Point != EFFECTIVE_ROW)
    {
        for (i = Divide_Point; i > ImageH / 6; i--)
        {
            if (imageLine.Exist_Left[i] && imageLine.Point_Left[i] > L_Max)
            {
                Mid_Point = i;
                L_Max = imageLine.Point_Left[i];
            }

            if (!imageLine.Exist_Left[i] || imageLine.Point_Left[i] < L_Max)
            {
                break;
            }
        }
    }
    else if (!Flag_Left_Ring_Find)
    {
        for (i = ImageH - 6; i > ImageH / 6; i--)
        {
            if (imageLine.Exist_Left[i] && imageLine.Point_Left[i] < L_Max)
            {
                Mid_Point = i;
                L_Max = imageLine.Point_Left[i];
            }

            if (!imageLine.Exist_Left[i] || imageLine.Point_Left[i] > L_Max)
            {
                break;
            }
        }
    }

    if ((Mid_Point == EFFECTIVE_ROW || imageLine.Point_Left[Mid_Point] <= 2 || !RightLine_Check(Mid_Point))
        && !Flag_Left_Ring_Find)
    {
        return;
    }
    //识别到圆环之后丢失转折点
    if ((Mid_Point == EFFECTIVE_ROW || imageLine.Point_Left[Mid_Point] <= 2)
        && Down_Point != EFFECTIVE_ROW && Flag_Right_Ring_Find)
    {
        float k_Left = 0, b_Left = 0;
        k_Left = ((float)imageLine.Point_Left[Down_Point + 1] - (float)5)
            / ((float)Down_Point + 1 - (float)59);
        b_Left = (float)5 - k_Left * 59;
        for (i = Down_Point; i >= 1; i--)
        {
            imageLine.Exist_Left[i] = 1;
            imageLine.Point_Left[i] = getLineValue(i, k_Left, b_Left);
        }
    }
    /************************环岛上方通道滤波***********************/
    for (i = Mid_Point - 1; i > EFFECTIVE_ROW; i--)
    {
        if (!imageLine.Exist_Left[i] || (imageLine.Point_Left[i] - imageLine.Point_Left[Mid_Point] < -5))
        {
            for (j = i - 1; j > EFFECTIVE_ROW; j--)
            {
                if (imageLine.Exist_Left[j] && (imageLine.Point_Left[j] - imageLine.Point_Left[Mid_Point] < -5))
                {
                    imageLine.Exist_Left[j] = 0;
                }
                if (imageLine.Exist_Left[j] && (imageLine.Point_Left[Mid_Point] - imageLine.Point_Left[j] < -3))
                {
                    Top_Point = j;
                    break;
                }
            }
        }
        if (Top_Point != EFFECTIVE_ROW)
            break;
    }
    /************************状态转换***********************/
    if (Flag_Left_Ring_Find && Top_Point != EFFECTIVE_ROW && imageLine.Point_Left[Top_Point] > 2
        && (Down_Point == EFFECTIVE_ROW || imageLine.Point_Left[Down_Point] <= 2))
    {
        if (Mid_Point >= 32)
        {
            Flag_Left_Ring_Find = 0;
            Flag_Left_Ring_Turn = 1;
            return;
        }
    }
    /************************补线及滤波***********************/
    if (Down_Point != EFFECTIVE_ROW)//滤除拐点上方的左右边界点 防止导致中线拟合出问题
    {
        for (i = Down_Point; i > EFFECTIVE_ROW; i--)
        {
            if (imageLine.Point_Left[i] < imageLine.Point_Left[Mid_Point])
            {
                imageLine.Exist_Left[i] = 0;
            }
            if (imageLine.Point_Right[i] < imageLine.Point_Left[Mid_Point])
            {
                imageLine.Exist_Right[i] = 0;
            }
        }

        float k_Left = 0, b_Left = 0;//用上下拐点来补线
        k_Left = ((float)imageLine.Point_Left[Down_Point] - (float)imageLine.Point_Left[Mid_Point])
            / ((float)Down_Point - (float)Mid_Point);
        b_Left = (float)imageLine.Point_Left[Mid_Point] - k_Left * Mid_Point;
        if (k_Left <= -1.4 || k_Left >= -0.5)
        {
            k_Left = ((float)5 - (float)imageLine.Point_Left[Mid_Point])
                / ((float)59 - (float)Mid_Point);
            b_Left = (float)imageLine.Point_Left[Mid_Point] - k_Left * Mid_Point;
        }
        for (i = Down_Point; i > Mid_Point; i--)
        {
            imageLine.Exist_Left[i] = 1;
            imageLine.Point_Left[i] = getLineValue(i, k_Left, b_Left);
            Flag_Left_Ring_Find = 1;
        }
    }
    else //下方拐点丢失 从上方拐点进行向下拉线补线
    {
        for (i = ImageH; i > Mid_Point; i--)
        {
            imageLine.Exist_Left[i] = 0;
            if (imageLine.Point_Right[i] < imageLine.Point_Left[Mid_Point])
                imageLine.Exist_Right[i] = 0;
        }

        for (i = Mid_Point; i > 1; i--)
        {
            if (imageLine.Point_Left[i] < imageLine.Point_Left[Mid_Point])
            {
                imageLine.Exist_Left[i] = 0;
            }
            if (imageLine.Point_Right[i] < imageLine.Point_Left[Mid_Point])
            {
                imageLine.Exist_Right[i] = 0;
            }
        }

        float k_left = 0, b_left = 0;
        k_left = ((float)5 - (float)imageLine.Point_Left[Mid_Point])
            / ((float)59 - (float)Mid_Point);
        b_left = (float)imageLine.Point_Left[Mid_Point] - k_left * Mid_Point;

        //开始补线
        for (i = ImageH - 1; i >= Mid_Point; i--)
        {
            imageLine.Exist_Left[i] = 1;
            imageLine.Point_Left[i] = getLineValue(i, k_left, b_left);
            Flag_Left_Ring_Find = 1;
        }
    }
}
void Left_Ring_Turn(void)
{
    int i = 0, j = 0, k = 0;
    int Top_Point = ImageW, Top_Point_row = EFFECTIVE_ROW, Divide_Point = EFFECTIVE_ROW;;
    int Endpoint_R = ImageW, Startline_R = 0;

    if (!Flag_Left_Ring_Turn)
    {
        return;
    }
    /************************找环岛上端点***********************/
    for (i = ImageH - 8; i > ImageH / 5; i--)
    {
        if (imageLine.Exist_Left[i] && !imageLine.Exist_Left[i - 1] && !imageLine.Exist_Left[i - 2])
        {
            Divide_Point = i;
            break;
        }
        if (!imageLine.Exist_Left[i] && !imageLine.Exist_Left[i - 1] && !imageLine.Exist_Left[i - 2] &&
            !imageLine.Exist_Left[i - 3] && !imageLine.Exist_Left[i - 4] && !imageLine.Exist_Left[i - 5])
            break;
    }
    /************************找环岛上直道断点***********************/
    if (Divide_Point != EFFECTIVE_ROW)
    {
        for (i = Divide_Point - 1; i > EFFECTIVE_ROW; i--)
        {
            if (imageLine.Exist_Left[i] && imageLine.Exist_Left[i - 1] &&
                imageLine.Point_Left[i] > ImageW / 4 && imageLine.Point_Left[i - 1] > ImageW / 4 &&
                (imageLine.Point_Left[i] - imageLine.Point_Left[i - 1] >= -2))
            {
                Top_Point_row = i;
                Top_Point = imageLine.Point_Left[i];
                break;
            }
        }

        for (i = Top_Point_row + 5; i > Top_Point_row; i--)
        {
            for (j = Top_Point; j > Top_Point - 15; j--)
            {
                if (isWhite(i, j) && !isWhite(i, j - 1))
                {
                    for (k = j - 1; k > Top_Point - 15; k--)
                    {
                        if (!isWhite(i, k) && isWhite(i, k - 1))
                        {
                            Top_Point_row = i;
                            Top_Point = j;
                            break;
                        }
                    }
                }
                if (Top_Point_row == i)
                    break;
            }
            if (Top_Point_row == i)
                break;
        }
    }
    else
    {
        for (i = ImageH - 8; i > EFFECTIVE_ROW; i--)
        {
            if (imageLine.Exist_Left[i] && imageLine.Exist_Left[i - 1] &&
                imageLine.Point_Left[i] > ImageW / 4 && imageLine.Point_Left[i - 1] > ImageW / 4 &&
                (imageLine.Point_Left[i] - imageLine.Point_Left[i - 1] >= -2))
            {
                Top_Point_row = i;
                Top_Point = imageLine.Point_Left[i];
                break;
            }
        }

        for (i = Top_Point_row + 5; i > Top_Point_row; i--)
        {
            for (j = Top_Point; j > Top_Point - 15; j--)
            {
                if (isWhite(i, j) && !isWhite(i, j - 1))
                {
                    for (k = j - 1; k > Top_Point - 15; k--)
                    {
                        if (!isWhite(i, k) && isWhite(i, k - 1))
                        {
                            Top_Point_row = i;
                            Top_Point = j;
                            break;
                        }
                    }
                }
                if (Top_Point_row == i)
                    break;
            }
            if (Top_Point_row == i)
                break;
        }
    }
    /************************状态转换***********************/
    for (i = ImageH - 5; i > EFFECTIVE_ROW; i--)
    {
        if (imageLine.Exist_Right[i])
        {
            Startline_R = i;
            break;
        }
    }

    for (j = Startline_R - 1; j > EFFECTIVE_ROW; j--)
    {
        if (imageLine.Exist_Right[j] && imageLine.Exist_Right[j + 1] && imageLine.Exist_Right[j - 1] &&
            (abs((imageLine.Point_Right[j - 1] + imageLine.Point_Right[j + 1]) / 2 - imageLine.Point_Right[j]) < 10))
        {
            Endpoint_R = imageLine.Point_Right[j];
        }
        else
            break;
    }
    if (Endpoint_R < ImageW / 2 && Startline_R >= 40)
    {
        Flag_Left_Ring_Turn = 0;
        Flag_Left_Ring_Out = 1;
        return;
    }

    if (Top_Point_row == EFFECTIVE_ROW)
    {
        return;
    }
    /************************补线及滤波***********************/
    for (i = ImageH - 1; i >= 1; i--)
    {
        imageLine.Exist_Right[i] = 0;
    }
    for (i = Top_Point_row; i >= 1; i--)
    {
        imageLine.Exist_Left[i] = 0;
    }

    float k_right = 0, b_right = 0;
    k_right = ((float)Constrain_Int((Top_Point + 30), -10, 104) - (float)Top_Point)
        / ((float)ImageH - (float)Top_Point_row);
    b_right = (float)Constrain_Int((Top_Point + 30), -10, 104) - k_right * ImageH;
    for (i = ImageH; i >= 1; i--)
    {
        if (i >= Top_Point_row)
        {
            imageLine.Exist_Right[i] = 1;
            imageLine.Point_Right[i] = k_right * i + b_right;
        }
        else
        {
            imageLine.Exist_Right[i] = 1;
            imageLine.Point_Right[i] = (3 * k_right) * i + (b_right - 2 * k_right * Top_Point_row);
        }
    }
}
void Left_Ring_Out(void)
{
    int i = 0, j = 0;
    int WhiteLine_cnt = 0;
    int turn_Point_Row = EFFECTIVE_ROW;
    int StartLine_L = ImageH;
    int LeftMend_Row = EFFECTIVE_ROW;

    if (!Flag_Left_Ring_Out)
        return;
    /************************找拐点***********************/
    for (i = ImageH - 10; i >= 10; i--)
    {
        if (imageLine.Exist_Right[i])
        {
            for (j = i + 1; j <= ImageH - 2; j++)
            {
                if (imageLine.Exist_Right[j] && imageLine.Point_Right[j] < imageLine.Point_Right[i])
                {
                    turn_Point_Row = j;
                    break;
                }
            }
        }
        if (turn_Point_Row != EFFECTIVE_ROW)
            break;
    }
    /************************找图像左侧最上白点***********************/
    for (i = ImageH - 5; i > EFFECTIVE_ROW; i--)
    {
        if (!isWhite(i - 2, 3) && !isWhite(i - 1, 3) && !isWhite(i, 3)
            && isWhite(i + 1, 3) && isWhite(i + 2, 3) && isWhite(i + 3, 3))
        {
            LeftMend_Row = i + 1;
            break;
        }
    }
    /************************找左边线起点***********************/
    for (i = ImageH - 10; i >= EFFECTIVE_ROW; i--)
    {
        if (!imageLine.Exist_Left[i] && !imageLine.Exist_Left[i - 1] && !imageLine.Exist_Left[i - 2]
            && !imageLine.Exist_Left[i - 3] && !imageLine.Exist_Left[i - 4])
        {
            for (j = i; j >= EFFECTIVE_ROW; j--)
            {
                if (imageLine.Exist_Left[j])
                {
                    StartLine_L = j;
                    break;
                }
            }
        }
        if (StartLine_L != ImageH)
        {
            break;
        }
    }
    /************************状态转换***********************/
    for (i = ImageH - 5; i > EFFECTIVE_ROW; i--)
    {
        if (imageLine.White_Num[i] == 94)
        {
            WhiteLine_cnt++;
        }
    }
    if (WhiteLine_cnt >= 20)
    {
        Flag_Left_Ring_Out = 0;
        Flag_Left_Ring_Out_Mend = 1;
        return;
    }
    /************************补线及滤波***********************/
    if (turn_Point_Row == EFFECTIVE_ROW)
    {
        return;
    }

    for (i = turn_Point_Row - 1; i >= 1; i--)
    {
        imageLine.Exist_Right[i] = 0;
    }

    for (i = StartLine_L; i >= 1; i--)
    {
        imageLine.Exist_Left[i] = 0;
    }

    float k_Right = 0, b_Right = 0;
    k_Right = ((float)imageLine.Point_Right[turn_Point_Row] - (float)3)
        / ((float)turn_Point_Row - (float)LeftMend_Row);
    b_Right = (float)imageLine.Point_Right[turn_Point_Row] - k_Right * turn_Point_Row;

    float k_right = 0, b_right = 0;
    short MendBasis_Right[2][5];
    int count = 0;
    for (i = turn_Point_Row; i <= turn_Point_Row + 10; i++)
    {
        if (imageLine.Exist_Right[i])
        {
            MendBasis_Right[0][count] = (short)i;
            MendBasis_Right[1][count] = (short)imageLine.Point_Right[i];
            count++;
        }
        if (count == 5)
            break;
    }
    if (count == 5)    //有5个点即可开始补线
        leastSquareMethod(MendBasis_Right[0], MendBasis_Right[1], 5, &k_right, &b_right);

    if (k_Right >= k_right)
    {
        for (i = turn_Point_Row; i >= LeftMend_Row; i--)
        {
            imageLine.Exist_Right[i] = 1;
            imageLine.Point_Right[i] = getLineValue(i, k_Right, b_Right);
        }
    }
    else
    {
        for (i = turn_Point_Row; i >= LeftMend_Row; i--)
        {
            imageLine.Exist_Right[i] = 1;
            imageLine.Point_Right[i] = getLineValue(i, k_right, b_right);
        }
    }
}
void Left_Ring_Out_Mend(void)
{
    int i = 0, j = 0;
    int Top_Point = EFFECTIVE_ROW, LeftMend_Row = EFFECTIVE_ROW, Startline_L = EFFECTIVE_ROW;
    int count_real = 0;
    float k_real = 0, b_real = 0;
    short real_dot[2][10];

    if (!Flag_Left_Ring_Out_Mend)
    {
        return;
    }
    /************************算右边线实际斜率***********************/
    for (i = 0; i < 2; i++)
    {
        for (j = 0; j < 10; j++)
        {
            real_dot[i][j] = 0;
        }
    }
    for (i = ImageH - 20; i >= EFFECTIVE_ROW; i--)
    {
        if (imageLine.Exist_Right[i] && (abs((imageLine.Point_Right[i - 1] + imageLine.Point_Right[i + 1]) / 2 - imageLine.Point_Right[i]) < 10))
        {
            real_dot[0][count_real] = (short)i;
            real_dot[1][count_real] = (short)imageLine.Point_Right[i];
            count_real++;
        }
        if (count_real == 10)
            break;
    }
    if (count_real == 10)
    {
        leastSquareMethod(real_dot[0], real_dot[1], 10, &k_real, &b_real);
    }
    if (!(count_real == 10 && k_real <= 3))
    {
        /************************函数Out补线的继续***********************/
        for (i = ImageH - 10; i >= EFFECTIVE_ROW; i--)
        {
            if (!isWhite(i - 2, 3) && !isWhite(i - 1, 3) && !isWhite(i, 3)
                && isWhite(i + 1, 3) && isWhite(i + 2, 3) && isWhite(i + 3, 3))
            {
                LeftMend_Row = i + 1;
                break;
            }
        }
        float k_Right = 0, b_Right = 0;
        k_Right = ((float)69 - (float)3)
            / ((float)ImageH - (float)LeftMend_Row);
        b_Right = (float)69 - k_Right * ImageH;
        for (i = ImageH; i >= 1; i--)
        {
            imageLine.Exist_Left[i] = 0;
            imageLine.Exist_Right[i] = 0;
        }
        for (i = ImageH; i >= 1; i--)
        {
            imageLine.Exist_Right[i] = 1;
            imageLine.Point_Right[i] = getLineValue(i, k_Right, b_Right);
        }
    }
    else
    {
        /************************找圆环上端点***********************/
        for (i = ImageH - 1; i > ImageH / 3; i--)
        {
            for (j = 1; j <= 5; j++)
            {
                Startline_L = i;
                if (!isWhite(i, j))
                {
                    Startline_L = EFFECTIVE_ROW;
                    break;
                }
            }
            if (Startline_L != EFFECTIVE_ROW)
                break;
        }
        for (i = Startline_L; i > ImageH / 4; i--)
        {
            if (imageLine.Exist_Left[i] && imageLine.Exist_Left[i - 1] &&
                imageLine.Exist_Left[i - 2] && imageLine.Exist_Left[i - 3] &&
                abs(imageLine.Point_Left[i - 1] - imageLine.Point_Left[i]) < 3 &&
                abs(imageLine.Point_Left[i - 2] - imageLine.Point_Left[i]) < 3 &&
                abs(imageLine.Point_Left[i - 3] - imageLine.Point_Left[i]) < 3)
            {
                Top_Point = i;
                break;
            }
        }
        /************************状态转换***********************/
        if (Top_Point > 45)
        {
            Flag_Left_Ring_Out_Mend = 0;
            Flag_Left_Ring_OnlyOnce = 0;
            Flag_Left_Ring_Clc = 1;
            return;
        }

        if (Top_Point == EFFECTIVE_ROW)
            return;
        /************************补线滤波***********************/
        for (i = ImageH; i > Top_Point; i--)
        {
            imageLine.Exist_Left[i] = 0;
        }

        float k_left = 0, b_left = 0;
        k_left = ((float)5 - (float)imageLine.Point_Left[Top_Point])
            / ((float)59 - (float)Top_Point);
        b_left = (float)imageLine.Point_Left[Top_Point] - k_left * Top_Point;

        //开始补线
        for (i = ImageH - 1; i >= Top_Point; i--)
        {
            imageLine.Exist_Left[i] = 1;
            imageLine.Point_Left[i] = getLineValue(i, k_left, b_left);
        }
    }
}


void Right_Ring(void)
{
    if (ForkRoad)
        return;
    if (Flag_Right_Ring_OnlyOnce)
    {
        Right_Ring_Find();
        Right_Ring_Turn();
        Right_Ring_Out();
        Right_Ring_Out_Mend();
    }
    if (Flag_Right_Ring_Find || Flag_Right_Ring_Turn || Flag_Right_Ring_Out || Flag_Right_Ring_Out_Mend)
    {
        R_Ring = 1;
    }
    else
        R_Ring = 0;
}
void Right_Ring_Find(void)
{
    int i = 0, j = 0;
    int Top_Point = EFFECTIVE_ROW, Mid_Point = EFFECTIVE_ROW, Down_Point = EFFECTIVE_ROW, Divide_Point = EFFECTIVE_ROW;
    int R_Min = ImageW;

    if (Flag_Right_Ring_Turn || Flag_Right_Ring_Out || Flag_Right_Ring_Out_Mend)//在圆环的后状态则不执行找圆环
        return;
    /************************找环岛下直道端点***********************/
    for (i = ImageH - 5; i > ImageH / 3; i--)
    {
        if ((i <= ImageH - 10) && (!imageLine.Exist_Right[i] && !imageLine.Exist_Right[i - 1] && !imageLine.Exist_Right[i - 2]
            && !imageLine.Exist_Right[i - 3] && !imageLine.Exist_Right[i - 4] && !imageLine.Exist_Right[i - 5]))
            break;
        if (imageLine.Exist_Right[i])
        {
            for (j = i + 1; j <= ImageH - 5; j++)
            {
                if (imageLine.Exist_Right[j] && imageLine.Point_Right[j] - imageLine.Point_Right[i] < 0)
                {
                    Down_Point = j;
                    break;
                }
            }
        }
        if (Down_Point != EFFECTIVE_ROW)
            break;
        if (imageLine.Exist_Right[i] && !imageLine.Exist_Right[i - 1] && !imageLine.Exist_Right[i - 2] && !imageLine.Exist_Right[i - 3]
            && !imageLine.Exist_Right[i - 4] && !imageLine.Exist_Right[i - 5] && !imageLine.Exist_Right[i - 6])
        {
            Down_Point = i;
            break;
        }
    }
    if (Down_Point != EFFECTIVE_ROW)
    {
        int e_cnt = 0;
        for (i = Down_Point; i >= Down_Point - 10; i--)
        {
            if (imageLine.Exist_Right[i] && imageLine.Point_Right[i] - imageLine.Point_Right[Down_Point] < 10
                && imageLine.Point_Right[i] - imageLine.Point_Right[Down_Point] > 0)
                e_cnt++;
        }
        if (e_cnt >= 3)
            Down_Point = EFFECTIVE_ROW;
    }
    /************************找环岛下方直道分界点***********************/
    if (Down_Point != EFFECTIVE_ROW)
    {
        for (i = Down_Point - 3; i > ImageH / 5; i--)
        {
            if (imageLine.Exist_Right[i] && imageLine.Point_Right[i] < imageLine.Point_Right[Down_Point])
            {
                Divide_Point = i;
                break;
            }
        }
    }
    else if (Flag_Right_Ring_Find)
    {
        for (j = ImageH - 6; j > ImageH / 5; j--)
        {
            if (imageLine.Exist_Right[j] && imageLine.Point_Right[j] < ImageW - 2)
            {
                Divide_Point = j;
                break;
            }
        }
    }
    else
        return;
    /************************找环岛转折点***********************/
    if (Divide_Point != EFFECTIVE_ROW)
    {
        for (i = Divide_Point; i > ImageH / 6; i--)
        {
            if (imageLine.Exist_Right[i] && imageLine.Point_Right[i] < R_Min)
            {
                Mid_Point = i;
                R_Min = imageLine.Point_Right[i];
            }

            if (!imageLine.Exist_Right[i] || imageLine.Point_Right[i] > R_Min)
            {
                break;
            }
        }
    }
    else if (!Flag_Right_Ring_Find)
    {
        for (i = ImageH - 6; i > ImageH / 6; i--)
        {
            if (imageLine.Exist_Right[i] && imageLine.Point_Right[i] < R_Min)
            {
                Mid_Point = i;
                R_Min = imageLine.Point_Right[i];
            }

            if (!imageLine.Exist_Right[i] || imageLine.Point_Right[i] > R_Min)
            {
                break;
            }
        }
    }

    if ((Mid_Point == EFFECTIVE_ROW || imageLine.Point_Right[Mid_Point] >= ImageW - 2 || !LeftLine_Check(Mid_Point))
        && !Flag_Right_Ring_Find)
    {
        return;
    }
    //识别到右圆环之后丢失转折点
    if ((Mid_Point == EFFECTIVE_ROW || imageLine.Point_Right[Mid_Point] >= ImageW - 2)
        && Down_Point != EFFECTIVE_ROW && Flag_Right_Ring_Find)
    {
        float k_Right = 0, b_Right = 0;
        k_Right = ((float)imageLine.Point_Right[Down_Point] - (float)89)
            / ((float)Down_Point - (float)ImageH);
        b_Right = (float)89 - k_Right * ImageH;
        for (i = Down_Point; i >= 1; i--)
        {
            imageLine.Exist_Right[i] = 1;
            imageLine.Point_Right[i] = getLineValue(i, k_Right, b_Right);
        }
    }
    /************************环岛上方通道滤波***********************/
    for (i = Mid_Point - 1; i > EFFECTIVE_ROW; i--)
    {
        if (!imageLine.Exist_Right[i] || (imageLine.Point_Right[i] - imageLine.Point_Right[Mid_Point] > 5))
        {
            for (j = i - 1; j > EFFECTIVE_ROW; j--)
            {
                if (imageLine.Exist_Right[j] && (imageLine.Point_Right[j] - imageLine.Point_Right[Mid_Point] > 5))
                {
                    imageLine.Exist_Right[j] = 0;
                }
                if (imageLine.Exist_Right[j] && (imageLine.Point_Right[Mid_Point] - imageLine.Point_Right[j] > 3))
                {
                    Top_Point = j;
                    break;
                }
            }
        }
        if (Top_Point != EFFECTIVE_ROW)
            break;
    }
    /************************状态转换***********************/
    if (Flag_Right_Ring_Find&& Top_Point != EFFECTIVE_ROW && imageLine.Point_Right[Top_Point] < ImageW - 2
        && (Down_Point == EFFECTIVE_ROW || imageLine.Point_Right[Down_Point] >= ImageW - 2))
    {
        if (Mid_Point >= 32)
        {
            Flag_Right_Ring_Find = 0;
            Flag_Right_Ring_Turn = 1;
            return;
        }
    }
    /************************补线及滤波***********************/
    if (Down_Point != EFFECTIVE_ROW)//滤除拐点上方的左右边界点 防止导致中线拟合出问题
    {
        for (i = Down_Point; i > EFFECTIVE_ROW; i--)
        {
            if (imageLine.Point_Left[i] > imageLine.Point_Right[Mid_Point])
            {
                imageLine.Exist_Left[i] = 0;
            }
            if (imageLine.Point_Right[i] > imageLine.Point_Right[Mid_Point])
            {
                imageLine.Exist_Right[i] = 0;
            }
        }

        float k_Right = 0, b_Right = 0;//用上下拐点来补线
        k_Right = ((float)imageLine.Point_Right[Down_Point] - (float)imageLine.Point_Right[Mid_Point])
            / ((float)Down_Point - (float)Mid_Point);
        b_Right = (float)imageLine.Point_Right[Mid_Point] - k_Right * Mid_Point;
        if (k_Right >= 1.4 || k_Right <= 0.5)
        {
            k_Right = ((float)89 - (float)imageLine.Point_Right[Mid_Point])
                / ((float)59 - (float)Mid_Point);
            b_Right = (float)imageLine.Point_Right[Mid_Point] - k_Right * Mid_Point;
        }
        for (i = Down_Point; i > Mid_Point; i--)
        {
            imageLine.Exist_Right[i] = 1;
            imageLine.Point_Right[i] = getLineValue(i, k_Right, b_Right);
            Flag_Right_Ring_Find = 1;
        }
    }
    else //下方拐点丢失 从上方拐点进行向下拉线补线
    {
        for (i = ImageH; i > Mid_Point; i--)
        {
            imageLine.Exist_Right[i] = 0;
            if (imageLine.Point_Left[i] > imageLine.Point_Right[Mid_Point])
                imageLine.Exist_Left[i] = 0;
        }

        for (i = Mid_Point; i > 1; i--)
        {
            if (imageLine.Point_Left[i] > imageLine.Point_Right[Mid_Point])
                imageLine.Exist_Left[i] = 0;
            if (imageLine.Point_Right[i] > imageLine.Point_Right[Mid_Point])
                imageLine.Exist_Right[i] = 0;
        }

        float k_right = 0, b_right = 0;
        k_right = ((float)89 - (float)imageLine.Point_Right[Mid_Point])
            / ((float)ImageH - (float)Mid_Point);
        b_right = (float)imageLine.Point_Right[Mid_Point] - k_right * Mid_Point;

        //开始补线
        for (i = ImageH; i >= Mid_Point; i--)
        {
            imageLine.Exist_Right[i] = 1;
            imageLine.Point_Right[i] = getLineValue(i, k_right, b_right);
            Flag_Right_Ring_Find = 1;
        }
    }
}
void Right_Ring_Turn(void)
{
    int i = 0, j = 0, k = 0;
    int Top_Point = ImageW, Top_Point_row = EFFECTIVE_ROW, Divide_Point = EFFECTIVE_ROW;
    int Endpoint_L = 0, Startline_L = EFFECTIVE_ROW;

    if (!Flag_Right_Ring_Turn)
        return;
    /************************找环岛上端点***********************/
    for (i = ImageH - 8; i > ImageH / 5; i--)
    {
        if (imageLine.Exist_Right[i] && !imageLine.Exist_Right[i - 1] && !imageLine.Exist_Right[i - 2])
        {
            Divide_Point = i;
            break;
        }
        if (!imageLine.Exist_Right[i] && !imageLine.Exist_Right[i - 1] && !imageLine.Exist_Right[i - 2] &&
            !imageLine.Exist_Right[i - 3] && !imageLine.Exist_Right[i - 4] && !imageLine.Exist_Right[i - 5])
            break;
    }
    /************************找环岛上直道断点***********************/
    if (Divide_Point != EFFECTIVE_ROW)
    {
        for (i = Divide_Point - 1; i > EFFECTIVE_ROW; i--)
        {
            if (imageLine.Exist_Right[i] && imageLine.Exist_Right[i - 1] &&
                imageLine.Point_Right[i] < 3 * ImageW / 4 && imageLine.Point_Right[i - 1] < 3 * ImageW / 4 &&
                (imageLine.Point_Right[i] - imageLine.Point_Right[i - 1] <= 2))
            {
                Top_Point_row = i;
                Top_Point = imageLine.Point_Right[i];
                break;
            }
        }

        for (i = Top_Point_row + 5; i > Top_Point_row; i--)
        {
            for (j = Top_Point; j < Top_Point + 15; j++)
            {
                if (isWhite(i, j) && !isWhite(i, j + 1))
                {
                    for (k = j + 1; k < Top_Point + 15; k++)
                    {
                        if (!isWhite(i, k) && isWhite(i, k + 1))
                        {
                            Top_Point_row = i;
                            Top_Point = j;
                            break;
                        }
                    }
                }
                if (Top_Point_row == i)
                    break;
            }
            if (Top_Point_row == i)
                break;
        }
    }
    else
    {
        for (i = ImageH - 8; i > EFFECTIVE_ROW; i--)
        {
            if (imageLine.Exist_Right[i] && imageLine.Exist_Right[i - 1] &&
                imageLine.Point_Right[i] < 3 * ImageW / 4 && imageLine.Point_Right[i - 1] < 3 * ImageW / 4 &&
                (imageLine.Point_Right[i] - imageLine.Point_Right[i - 1] <= 2))
            {
                Top_Point_row = i;
                Top_Point = imageLine.Point_Right[i];
                break;
            }
        }
        for (i = Top_Point_row + 5; i > Top_Point_row; i--)
        {
            for (j = Top_Point; j < Top_Point + 15; j++)
            {
                if (isWhite(i, j) && !isWhite(i, j + 1))
                {
                    for (k = j + 1; k < Top_Point + 15; k++)
                    {
                        if (!isWhite(i, k) && isWhite(i, k + 1))
                        {
                            Top_Point_row = i;
                            Top_Point = j;
                            break;
                        }
                    }
                }
                if (Top_Point_row == i)
                    break;
            }
            if (Top_Point_row == i)
                break;
        }
    }
    /************************状态转换***********************/
    for (i = ImageH - 5; i > EFFECTIVE_ROW; i--)
    {
        if (imageLine.Exist_Left[i])
        {
            Startline_L = i;
            break;
        }
    }

    for (j = Startline_L - 1; j > EFFECTIVE_ROW; j--)
    {
        if (imageLine.Exist_Left[j] && imageLine.Exist_Left[j + 1] && imageLine.Exist_Left[j - 1] &&
            (abs((imageLine.Point_Left[j - 1] + imageLine.Point_Left[j + 1]) / 2 - imageLine.Point_Left[j]) < 10))
        {
            Endpoint_L = imageLine.Point_Left[j];
        }
        else
            break;
    }
    if (Startline_L >= 40 && Endpoint_L > ImageW / 2)
    {
        Flag_Right_Ring_Turn = 0;
        Flag_Right_Ring_Out = 1;
        return;
    }

    if (Top_Point_row == EFFECTIVE_ROW)
    {
        return;
    }
    /************************补线及滤波***********************/
    for (i = ImageH; i >= 1; i--)
    {
        imageLine.Exist_Left[i] = 0;
    }
    for (i = Top_Point_row; i >= 1; i--)
    {
        imageLine.Exist_Right[i] = 0;
    }

    float k_left, b_left;
    k_left = ((float)Constrain_Int((Top_Point - 30), -10, 104) - (float)Top_Point)
        / ((float)ImageH - (float)Top_Point_row);
    b_left = (float)Constrain_Int((Top_Point - 30), -10, 104) - k_left * ImageH;

    for (i = ImageH; i >= 1; i--)
    {
        if (i >= Top_Point_row)
        {
            imageLine.Exist_Left[i] = 1;
            imageLine.Point_Left[i] = k_left * i + b_left;
        }
        else
        {
            imageLine.Exist_Left[i] = 1;
            imageLine.Point_Left[i] = (3 * k_left) * i + (b_left - 2 * k_left * Top_Point_row);
        }
    }
}
void Right_Ring_Out(void)
{
    int i = 0, j = 0;
    int WhiteLine_cnt = 0;
    int turn_Point_Row = EFFECTIVE_ROW;
    int StartLine_R = ImageH;
    int RightMend_Row = EFFECTIVE_ROW;

    if (!Flag_Right_Ring_Out)
        return;
    /************************找拐点***********************/
    for (i = ImageH - 2; i >= 10; i--)
    {
        if (imageLine.Exist_Left[i])
        {
            for (j = i + 1; j <= ImageH - 2; j++)
            {
                if (imageLine.Exist_Left[j] && imageLine.Point_Left[j] > imageLine.Point_Left[i])
                {
                    turn_Point_Row = j;
                    break;
                }
            }
        }
        if (turn_Point_Row != EFFECTIVE_ROW)
            break;
    }
    /************************找图像右侧最上白点***********************/
    for (i = ImageH - 5; i > EFFECTIVE_ROW; i--)
    {
        if (!isWhite(i - 2, 91) && !isWhite(i - 1, 91) && !isWhite(i, 91)
            && isWhite(i + 1, 91) && isWhite(i + 2, 91) && isWhite(i + 3, 91))
        {
            RightMend_Row = i + 1;
            break;
        }
    }
    /************************找右边线起点及滤波***********************/
    for (i = ImageH - 10; i >= EFFECTIVE_ROW; i--)
    {
        if (!imageLine.Exist_Right[i] && !imageLine.Exist_Right[i - 1] && !imageLine.Exist_Right[i - 2]
            && !imageLine.Exist_Right[i - 3] && !imageLine.Exist_Right[i - 4])
        {
            for (j = i; j >= EFFECTIVE_ROW; j--)
            {
                if (imageLine.Exist_Right[j])
                {
                    StartLine_R = j;
                    break;
                }
            }
        }
        if (StartLine_R != ImageH)
        {
            break;
        }
    }
    /************************状态转换***********************/
    for (i = ImageH - 5; i > EFFECTIVE_ROW; i--)
    {
        if (imageLine.White_Num[i] == 94)
        {
            WhiteLine_cnt++;
        }
    }
    if (WhiteLine_cnt >= 20)
    {
        Flag_Right_Ring_Out = 0;
        Flag_Right_Ring_Out_Mend = 1;
        return;
    }
    /************************补线及滤波***********************/
    if (turn_Point_Row == EFFECTIVE_ROW)
    {
        return;
    }

    for (i = turn_Point_Row - 1; i >= 1; i--)
    {
        imageLine.Exist_Left[i] = 0;
    }
    for (i = StartLine_R; i >= 1; i--)
    {
        imageLine.Exist_Right[i] = 0;
    }

    float k_Left = 0, b_Left = 0;
    k_Left = ((float)imageLine.Point_Left[turn_Point_Row] - (float)91)
        / ((float)turn_Point_Row - (float)RightMend_Row);
    b_Left = (float)imageLine.Point_Left[turn_Point_Row] - k_Left * turn_Point_Row;

    float k_left = 0, b_left = 0;
    short MendBasis_Left[2][5];
    int count = 0;
    for (i = turn_Point_Row; i <= turn_Point_Row + 10; i++)
    {
        if (imageLine.Exist_Left[i])
        {
            MendBasis_Left[0][count] = (short)i;
            MendBasis_Left[1][count] = (short)imageLine.Point_Left[i];
            count++;
        }
        if (count == 5)
            break;
    }
    if (count == 5)    //有5个点即可开始补线
        leastSquareMethod(MendBasis_Left[0], MendBasis_Left[1], 5, &k_left, &b_left);


    if (k_Left <= k_left)
    {
        for (i = turn_Point_Row; i >= RightMend_Row; i--)
        {
            imageLine.Exist_Left[i] = 1;
            imageLine.Point_Left[i] = getLineValue(i, k_Left, b_Left);
        }
    }
    else
    {
        for (i = turn_Point_Row; i >= RightMend_Row; i--)
        {
            imageLine.Exist_Left[i] = 1;
            imageLine.Point_Left[i] = getLineValue(i, k_left, b_left);
        }
    }
}
void Right_Ring_Out_Mend(void)
{
    int i = 0, j = 0;
    int Top_Point = EFFECTIVE_ROW, RightMend_Row = EFFECTIVE_ROW, Startline_R = EFFECTIVE_ROW;
    int count_real = 0;
    float k_real = 0, b_real = 0;
    short real_dot[2][10];

    if (!Flag_Right_Ring_Out_Mend)
        return;
    /************************算左边线实际斜率***********************/
    for (i = 0; i < 2; i++)
    {
        for (j = 0; j < 10; j++)
        {
            real_dot[i][j] = 0;
        }
    }

    for (i = ImageH - 20; i >= EFFECTIVE_ROW; i--)
    {
        if (imageLine.Exist_Left[i] && (abs((imageLine.Point_Left[i - 1] + imageLine.Point_Left[i + 1]) / 2 - imageLine.Point_Left[i]) < 10))
        {
            real_dot[0][count_real] = (short)i;
            real_dot[1][count_real] = (short)imageLine.Point_Left[i];
            count_real++;
        }
        if (count_real == 10)
            break;
    }
    if (count_real == 10)
    {
        leastSquareMethod(real_dot[0], real_dot[1], 10, &k_real, &b_real);
    }
    if (!(count_real == 10 && k_real >= -3))
    {
        /************************函数Out补线的继续***********************/
        for (i = ImageH - 10; i > EFFECTIVE_ROW; i--)
        {
            if (!isWhite(i - 2, 91) && !isWhite(i - 1, 91) && !isWhite(i, 91)
                && isWhite(i + 1, 91) && isWhite(i + 2, 91) && isWhite(i + 3, 91))
            {
                RightMend_Row = i + 1;
                break;
            }
        }
        float k_Left = 0, b_Left = 0;
        k_Left = ((float)25 - (float)91)
            / ((float)ImageH - (float)RightMend_Row);
        b_Left = (float)25 - k_Left * ImageH;
        for (i = ImageH; i >= 1; i--)
        {
            imageLine.Exist_Left[i] = 0;
            imageLine.Exist_Right[i] = 0;
        }
        for (i = ImageH; i >= 1; i--)
        {
            imageLine.Exist_Left[i] = 1;
            imageLine.Point_Left[i] = getLineValue(i, k_Left, b_Left);
        }
    }
    else
    {
        /************************找圆环上端点***********************/
        for (i = ImageH - 1; i > ImageH / 3; i--)
        {
            for (j = ImageW - 5; j <= ImageW - 1; j++)
            {
                Startline_R = i;
                if (!isWhite(i, j))
                {
                    Startline_R = EFFECTIVE_ROW;
                    break;
                }
            }
            if (Startline_R != EFFECTIVE_ROW)
                break;
        }

        for (i = Startline_R; i > ImageH / 4; i--)
        {
            if (imageLine.Exist_Right[i] && imageLine.Exist_Right[i - 1] &&
                imageLine.Exist_Right[i - 2] && imageLine.Exist_Right[i - 3] &&
                abs(imageLine.Point_Right[i] - imageLine.Point_Right[i - 1]) < 3 &&
                abs(imageLine.Point_Right[i] - imageLine.Point_Right[i - 2]) < 3 &&
                abs(imageLine.Point_Right[i] - imageLine.Point_Right[i - 3]) < 3)
            {
                Top_Point = i;
                break;
            }
        }
        /************************状态转换***********************/
        if (Top_Point > 45)
        {
            Flag_Right_Ring_Out_Mend = 0;
            Flag_Right_Ring_OnlyOnce = 0;
            Flag_Right_Ring_Clc = 1;
        }

        if (Top_Point == EFFECTIVE_ROW)
            return;
        /************************补线滤波***********************/
        for (i = ImageH - 1; i > Top_Point; i--)
        {
            imageLine.Exist_Right[i] = 0;
        }

        float k_right = 0, b_right = 0;
        k_right = ((float)89 - (float)imageLine.Point_Right[Top_Point])
            / ((float)ImageH - (float)Top_Point);
        b_right = (float)imageLine.Point_Right[Top_Point] - k_right * Top_Point;

        //开始补线
        for (i = ImageH; i >= Top_Point; i--)
        {
            imageLine.Exist_Right[i] = 1;
            imageLine.Point_Right[i] = getLineValue(i, k_right, b_right);
        }
    }
}
/*****************************************************往返*********************************************************************/
void T_Left(void)
{
    if (Flag_Right_Ring_Clc && Flag_T_Left_OnlyOnce)
    {
        T_Left_Find();
        T_Left_Turn();
        T_Left_Out();
    }
    if (Flag_T_Left_Find || Flag_T_Left_Turn || Flag_T_Left_Out || Flag_T_Left_Lostline)
    {
        T_Left_Road = 1;
    }
    else
    {
        T_Left_Road = 0;
    }
}
void T_Left_Find(void)
{
    int i = 0, j = 0;
    int Top_Point = EFFECTIVE_ROW, Down_Point = EFFECTIVE_ROW, Divide_Point = EFFECTIVE_ROW, black_line = EFFECTIVE_ROW;
    int R_Min = ImageW;

    if (Flag_T_Left_Turn || Flag_T_Left_Out)
        return;
    /************************找下直道端点***********************/
    for (i = ImageH - 1; i > ImageH / 5; i--)
    {
        if ((i <= ImageH - 10) && !imageLine.Exist_Right[i] && !imageLine.Exist_Right[i - 1] && !imageLine.Exist_Right[i - 2]
            && !imageLine.Exist_Right[i - 3] && !imageLine.Exist_Right[i - 4] && !imageLine.Exist_Right[i - 5])
            break;
        if (imageLine.Exist_Right[i] && imageLine.Exist_Right[i + 1])
        {
            for (j = i + 1; j <= ImageH - 3; j++)
            {
                if (imageLine.Exist_Right[j] && imageLine.Point_Right[j] - imageLine.Point_Right[i] < 0)
                {
                    Down_Point = j;
                }
                if (Down_Point != EFFECTIVE_ROW)
                {
                    break;
                }
            }
        }
        if (Down_Point != EFFECTIVE_ROW)
        {
            break;
        }
        if (imageLine.Exist_Right[i] && !imageLine.Exist_Right[i - 1] && !imageLine.Exist_Right[i - 2]
            && !imageLine.Exist_Right[i - 3] && !imageLine.Exist_Right[i - 4])
        {
            Down_Point = i;
            break;
        }
    }
    if (Down_Point != EFFECTIVE_ROW)
    {
        int e_cnt = 0;
        for (i = Down_Point; i >= Down_Point - 10; i--)
        {
            if (imageLine.Exist_Right[i] && imageLine.Point_Right[i] - imageLine.Point_Right[Down_Point] < 10
                && imageLine.Point_Right[i] - imageLine.Point_Right[Down_Point] > 0)
            {
                e_cnt++;
            }
        }
        if (e_cnt > 2)
            Down_Point = EFFECTIVE_ROW;
    }
    /************************找分界点***********************/
    if (Down_Point != EFFECTIVE_ROW)
    {
        for (i = Down_Point - 10; i >= ImageH / 6; i--)
        {
            if (imageLine.Exist_Right[i])
            {
                Divide_Point = i;
                break;
            }
        }
    }
    else if (Flag_T_Left_Find)
    {
        for (i = ImageH - 6; i >= ImageH / 6; i--)
        {
            if (imageLine.Exist_Right[i])
            {
                Divide_Point = i;
                break;
            }
        }
    }
    else
        return;
    /************************找上端点***********************/
    if (Down_Point != EFFECTIVE_ROW)
    {
        for (i = Divide_Point; i >= ImageH / 6; i--)
        {
            if (imageLine.Exist_Right[i] &&
                imageLine.Point_Right[i] < imageLine.Point_Right[Down_Point])
            {
                for (j = i; j >= ImageH / 6; j--)
                {
                    if (imageLine.Exist_Right[j] && imageLine.Point_Right[j] < R_Min)
                    {
                        Top_Point = j;
                        R_Min = imageLine.Point_Right[j];
                    }
                    if (!imageLine.Exist_Right[j] || imageLine.Point_Right[j] > R_Min ||
                        imageLine.Point_Right[j] >= imageLine.Point_Right[Down_Point])
                    {
                        break;
                    }
                }
            }
            if (Top_Point != EFFECTIVE_ROW)
                break;
        }
    }
    else
    {
        for (i = Divide_Point; i > ImageH / 6; i--)
        {
            if (imageLine.Exist_Right[i] && imageLine.Point_Right[i] < R_Min)
            {
                Top_Point = i;
                R_Min = imageLine.Point_Right[i];
            }
            if (!imageLine.Exist_Right[i] || imageLine.Point_Right[i] > R_Min)
            {
                break;
            }
        }
    }

    if ((Top_Point == EFFECTIVE_ROW || !LeftLine_Check(Top_Point))
        && !Flag_T_Left_Find)
    {
        return;
    }
    /************************状态转换***********************/
    if (Down_Point == EFFECTIVE_ROW)
    {
        for (i = ImageH - 6; i >= ImageH / 4; i--)
        {
            black_line = i;
            for (j = ImageW - 1; j >= ImageW - 10; j--)
            {
                if (isWhite(i, j))
                {
                    black_line = EFFECTIVE_ROW;
                    break;
                }
            }
            if (black_line != EFFECTIVE_ROW)
                break;
        }
    }

    if (Flag_T_Left_Find && Down_Point == EFFECTIVE_ROW &&
        black_line >= 45)
    {
        Flag_T_Left_Find = 0;
        Flag_T_Left_Turn = 1;
        return;
    }
    /************************补线及滤线***********************/
    if (Down_Point != EFFECTIVE_ROW)//滤除拐点上方的左右边界点 防止导致中线拟合出问题
    {
        for (i = Down_Point; i > EFFECTIVE_ROW; i--)
        {
            if (imageLine.Point_Left[i] > imageLine.Point_Right[Top_Point])
            {
                imageLine.Exist_Left[i] = 0;
            }
            if (imageLine.Point_Right[i] > imageLine.Point_Right[Top_Point])
            {
                imageLine.Exist_Right[i] = 0;
            }
        }

        float k_Right = 0, b_Right = 0;//用上下拐点来补线
        k_Right = ((float)imageLine.Point_Right[Down_Point] - (float)imageLine.Point_Right[Top_Point])
            / ((float)Down_Point - (float)Top_Point);
        b_Right = (float)imageLine.Point_Right[Top_Point] - k_Right * Top_Point;
        for (i = Down_Point; i > Top_Point; i--)
        {
            imageLine.Exist_Right[i] = 1;
            imageLine.Point_Right[i] = getLineValue(i, k_Right, b_Right);
            Flag_T_Left_Find = 1;
        }
    }
    else //下方拐点丢失 从上方拐点进行向下拉线补线
    {
        for (i = ImageH; i > Top_Point; i--)
        {
            imageLine.Exist_Right[i] = 0;
        }
        float k_right = 0, b_right = 0;
        k_right = ((float)89 - (float)imageLine.Point_Right[Top_Point])
            / ((float)ImageH - (float)Top_Point);
        b_right = (float)imageLine.Point_Right[Top_Point] - k_right * Top_Point;

        //开始补线
        for (i = ImageH; i > Top_Point; i--)
        {
            imageLine.Exist_Right[i] = 1;
            imageLine.Point_Right[i] = getLineValue(i, k_right, b_right);
            Flag_T_Left_Find = 1;
        }
    }
}
void T_Left_Turn(void)
{
    int i = 0, j = 0;
    int turn_Point_Row = EFFECTIVE_ROW;

    if (!Flag_T_Left_Turn)
        return;

    /************************找拐点***********************/
    for (i = ImageH - 5; i > ImageH / 5; i--)
    {
        if (imageLine.Exist_Left[i])
        {
            for (j = i + 1; j <= ImageH - 4; j++)
            {
                if (imageLine.Exist_Left[j] && imageLine.Point_Left[j] - imageLine.Point_Left[i] > 0)
                {
                    turn_Point_Row = j;
                }
                if (turn_Point_Row != EFFECTIVE_ROW)
                {
                    break;
                }
            }
        }
        if (turn_Point_Row != EFFECTIVE_ROW)
        {
            break;
        }
    }

    if (turn_Point_Row == EFFECTIVE_ROW)
    {
        return;
    }
    /************************滤波及补线***********************/
    for (i = turn_Point_Row - 1; i >= 1; i--)
    {
        imageLine.Exist_Left[i] = 0;
    }

    for (i = ImageH; i >= 1; i--)
    {
        imageLine.Exist_Right[i] = 0;
    }

    float k_Left = 0, b_Left = 0;
    short MendBasis_Left[2][7];
    int count = 0;

    for (i = turn_Point_Row + 1; i <= turn_Point_Row + 15; i++)
    {
        if (imageLine.Exist_Left[i])
        {
            MendBasis_Left[0][count] = (short)i;
            MendBasis_Left[1][count] = (short)imageLine.Point_Left[i];
            count++;
        }
        if (count == 7)
            break;
    }
    if (count == 7)    //有5个点即可开始补线
    {
        leastSquareMethod(MendBasis_Left[0], MendBasis_Left[1], 7, &k_Left, &b_Left);
        //开始补线
        for (i = turn_Point_Row - 1; i >= EFFECTIVE_ROW; i--)
        {
            imageLine.Exist_Left[i] = 1;
            imageLine.Point_Left[i] = getLineValue(i, k_Left, b_Left);
        }
    }
    /************************状态转换***********************/
    if (turn_Point_Row >= 35 || imageLine.Point_Left[turn_Point_Row] <= 10)
    {
        Flag_T_Left_Turn = 0;
        Flag_T_Left_Lostline = 1;
        Flag_T_Left_Out = 1;
    }
}
void T_Left_Out(void)
{
    int i = 0, j = 0;
    int Top_Point = EFFECTIVE_ROW;
    int count_real = 0;
    float k_real = 0, b_real = 0;
    short real_dot[2][20];

    if (!Flag_T_Left_Out)
    {
        return;
    }
    /************************实际右边线斜率***********************/
    for (i = 0; i < 2; i++)
    {
        for (j = 0; j < 20; j++)
        {
            real_dot[i][j] = 0;
        }
    }

    for (i = ImageH - 10; i >= EFFECTIVE_ROW; i--)
    {
        if (imageLine.Exist_Right[i])
        {
            real_dot[0][count_real] = (short)i;
            real_dot[1][count_real] = (short)imageLine.Point_Right[i];
            count_real++;
        }
        if (count_real == 20)
            break;
    }
    if (count_real == 20)
    {
        leastSquareMethod(real_dot[0], real_dot[1], 20, &k_real, &b_real);
    }
    /************************补线***********************/
    if (count_real == 20 && k_real > 0.5 && k_real < 1.2)
    {
        Flag_T_Left_Lostline = 0;

        for (i = ImageH - 3; i >= ImageH / 3; i--)
        {
            if (!imageLine.Exist_Left[i] && !imageLine.Exist_Left[i - 1] && !imageLine.Exist_Left[i - 2]
                && !imageLine.Exist_Left[i - 3] && !imageLine.Exist_Left[i - 4] && !imageLine.Exist_Left[i - 5])
            {
                for (j = i; j > EFFECTIVE_ROW; j--)
                {
                    if (imageLine.Exist_Left[j])
                    {
                        Top_Point = j;
                        break;
                    }
                }
            }
            if (Top_Point != EFFECTIVE_ROW)
                break;
        }

        if (Top_Point == EFFECTIVE_ROW)
            return;

        float k_Left = 0, b_Left = 0;
        k_Left = ((float)5 - (float)imageLine.Point_Left[Top_Point])
            / ((float)ImageH - (float)Top_Point);
        b_Left = (float)5 - k_Left * ImageH;
        for (i = ImageH; i >= Top_Point; i--)
        {
            imageLine.Exist_Left[i] = 1;
            imageLine.Point_Left[i] = getLineValue(i, k_Left, b_Left);
        }
    }
    if (count_real == 20 && k_real > 0.5 && k_real < 0.8)
    {
        Flag_T_Left_Out = 0;
        Flag_T_Left_OnlyOnce = 0;
        Flag_T_Left_Clc = 1;
    }

}

void T_Right(void)
{
    if (Flag_T_Right_OnlyOnce)
    {
        T_Right_Find();
        T_Right_Turn();
        T_Right_Out();
    }
    if (Flag_T_Right_Find || Flag_T_Right_Turn || Flag_T_Right_Out || Flag_T_Right_Lostline)
    {
        T_Right_Road = 1;
    }
    else
    {
        T_Right_Road = 0;
    }
}
void T_Right_Find(void)
{
    int i = 0, j = 0;
    int Top_Point = EFFECTIVE_ROW, Down_Point = EFFECTIVE_ROW, Divide_Point = EFFECTIVE_ROW, black_line = EFFECTIVE_ROW;
    int L_Max = 0;

    if (Flag_T_Right_Turn || Flag_T_Right_Out)
        return;
    /************************找下直道端点***********************/
    for (i = ImageH - 2; i > ImageH / 3; i--)
    {
        if (!imageLine.Exist_Left[i] && !imageLine.Exist_Left[i - 1] && !imageLine.Exist_Left[i - 2]
            && !imageLine.Exist_Left[i - 3] && !imageLine.Exist_Left[i - 4] && !imageLine.Exist_Left[i - 5])
            break;
        if (imageLine.Exist_Left[i] && imageLine.Exist_Left[i + 1])
        {
            for (j = i + 1; j <= ImageH - 3; j++)
            {
                if (imageLine.Exist_Left[j] && imageLine.Point_Left[j] - imageLine.Point_Left[i] > 0)
                {
                    Down_Point = j;
                }
                if (Down_Point != EFFECTIVE_ROW)
                {
                    break;
                }
            }
        }
        if (Down_Point != EFFECTIVE_ROW)
        {
            break;
        }
        if (imageLine.Exist_Left[i] && !imageLine.Exist_Left[i - 1] && !imageLine.Exist_Left[i - 2]
            && !imageLine.Exist_Left[i - 3] && !imageLine.Exist_Left[i - 4])
        {
            Down_Point = i;
            break;
        }
    }

    if (Down_Point != EFFECTIVE_ROW)
    {
        int e_cnt = 0;
        for (i = Down_Point; i >= Down_Point - 10; i--)
        {
            if (imageLine.Exist_Left[i] && imageLine.Point_Left[i] - imageLine.Point_Left[Down_Point] > -10
                && imageLine.Point_Left[i] - imageLine.Point_Left[Down_Point] < 0)
            {
                e_cnt++;
            }
        }
        if (e_cnt > 2)
            Down_Point = EFFECTIVE_ROW;
    }
    /************************找分界点***********************/
    if (Down_Point != EFFECTIVE_ROW)
    {
        for (i = Down_Point - 10; i >= ImageH / 4; i--)
        {
            if (imageLine.Exist_Left[i])
            {
                Divide_Point = i;
                break;
            }
        }
    }
    else if (Flag_T_Right_Find)
    {
        for (i = ImageH - 6; i >= ImageH / 4; i--)
        {
            if (imageLine.Exist_Left[i])
            {
                Divide_Point = i;
                break;
            }
        }
    }
    else
        return;
    /************************找上端点***********************/
    if (Down_Point != EFFECTIVE_ROW)
    {
        for (i = Divide_Point; i >= ImageH / 6; i--)
        {
            if (imageLine.Exist_Left[i] &&
                imageLine.Point_Left[i] > imageLine.Point_Left[Down_Point])
            {
                for (j = i; j >= ImageH / 6; j--)
                {
                    if (imageLine.Exist_Left[j] && imageLine.Point_Left[j] > L_Max)
                    {
                        Top_Point = j;
                        L_Max = imageLine.Point_Left[j];
                    }
                    if (!imageLine.Exist_Left[j] || imageLine.Point_Left[j] < L_Max ||
                        imageLine.Point_Left[j] <= imageLine.Point_Left[Down_Point])
                    {
                        break;
                    }
                }
            }
            if (Top_Point != EFFECTIVE_ROW)
                break;
        }
    }
    else
    {
        for (i = Divide_Point; i > ImageH / 6; i--)
        {
            if (imageLine.Exist_Left[i] && imageLine.Point_Left[i] > L_Max)
            {
                Top_Point = i;
                L_Max = imageLine.Point_Left[i];
            }
            if (!imageLine.Exist_Left[i] || imageLine.Point_Left[i] < L_Max)
            {
                break;
            }
        }
    }

    if ((Top_Point == EFFECTIVE_ROW || !RightLine_Check(Top_Point))
        && !Flag_T_Right_Find)
    {
        return;
    }
    /************************状态转换***********************/
    if (Down_Point == EFFECTIVE_ROW)
    {
        for (i = ImageH - 6; i >= ImageH / 4; i--)
        {
            black_line = i;
            for (j = 10; j >= 1; j--)
            {
                if (isWhite(i, j))
                {
                    black_line = EFFECTIVE_ROW;
                    break;
                }
            }
            if (black_line != EFFECTIVE_ROW)
                break;
        }
    }

    if (Flag_T_Right_Find && Down_Point == EFFECTIVE_ROW &&
        black_line >= 45)
    {
        Flag_T_Right_Find = 0;
        Flag_T_Right_Turn = 1;
        return;
    }
    /************************补线及滤线***********************/
    if (Down_Point != EFFECTIVE_ROW)//滤除拐点上方的左右边界点 防止导致中线拟合出问题
    {
        for (i = Down_Point; i > EFFECTIVE_ROW; i--)
        {
            if (imageLine.Point_Left[i] < imageLine.Point_Left[Top_Point])
            {
                imageLine.Exist_Left[i] = 0;
            }
            if (imageLine.Point_Right[i] < imageLine.Point_Left[Top_Point])
            {
                imageLine.Exist_Right[i] = 0;
            }
        }

        float k_Left = 0, b_Left = 0;//用上下拐点来补线
        k_Left = ((float)imageLine.Point_Left[Down_Point] - (float)imageLine.Point_Left[Top_Point])
            / ((float)Down_Point - (float)Top_Point);
        b_Left = (float)imageLine.Point_Left[Top_Point] - k_Left * Top_Point;
        for (i = Down_Point; i > Top_Point; i--)
        {
            imageLine.Exist_Left[i] = 1;
            imageLine.Point_Left[i] = getLineValue(i, k_Left, b_Left);
            Flag_T_Right_Find = 1;
        }
    }
    else //下方拐点丢失 从上方拐点进行向下拉线补线
    {
        for (i = ImageH; i > Top_Point; i--)
        {
            imageLine.Exist_Left[i] = 0;
        }
        float k_right = 0, b_right = 0;
        k_right = ((float)5 - (float)imageLine.Point_Left[Top_Point])
            / ((float)ImageH - (float)Top_Point);
        b_right = (float)imageLine.Point_Left[Top_Point] - k_right * Top_Point;

        //开始补线
        for (i = ImageH; i > Top_Point; i--)
        {
            imageLine.Exist_Left[i] = 1;
            imageLine.Point_Left[i] = getLineValue(i, k_right, b_right);
            Flag_T_Right_Find = 1;
        }
    }
}
void T_Right_Turn(void)
{
    int i = 0, j = 0;
    int turn_Point_Row = EFFECTIVE_ROW;

    if (!Flag_T_Right_Turn)
        return;

    /************************找拐点***********************/
    for (i = ImageH - 10; i > ImageH / 5; i--)
    {
        if (imageLine.Exist_Right[i])
        {
            for (j = i + 1; j <= ImageH - 10; j++)
            {
                if (imageLine.Exist_Right[j] && imageLine.Point_Right[j] - imageLine.Point_Right[i] < 0)
                {
                    turn_Point_Row = j;
                }
                if (turn_Point_Row != EFFECTIVE_ROW)
                {
                    break;
                }
            }
        }
        if (turn_Point_Row != EFFECTIVE_ROW)
        {
            break;
        }
    }

    if (turn_Point_Row == EFFECTIVE_ROW)
    {
        return;
    }
    /************************滤波及补线***********************/
    for (i = turn_Point_Row - 1; i >= 1; i--)
    {
        imageLine.Exist_Right[i] = 0;
    }

    for (i = ImageH; i >= 1; i--)
    {
        imageLine.Exist_Left[i] = 0;
    }

    float k_Right = 0, b_Right = 0;
    short MendBasis_Right[2][7];
    int count = 0;

    for (i = turn_Point_Row + 1; i <= turn_Point_Row + 15; i++)
    {
        if (imageLine.Exist_Right[i])
        {
            MendBasis_Right[0][count] = (short)i;
            MendBasis_Right[1][count] = (short)imageLine.Point_Right[i];
            count++;
        }
        if (count == 7)
            break;
    }
    if (count == 7)    //有5个点即可开始补线
    {
        leastSquareMethod(MendBasis_Right[0], MendBasis_Right[1], 7, &k_Right, &b_Right);
        //开始补线
        for (i = turn_Point_Row - 1; i >= EFFECTIVE_ROW; i--)
        {
            imageLine.Exist_Right[i] = 1;
            imageLine.Point_Right[i] = getLineValue(i, k_Right, b_Right);
        }
    }
    /************************状态转换***********************/
    if (turn_Point_Row >= 35 || imageLine.Point_Right[turn_Point_Row] >= ImageW - 10)
    {
        Flag_T_Right_Turn = 0;
        Flag_T_Right_Lostline = 1;
        Flag_T_Right_Out = 1;
    }
}
void T_Right_Out(void)
{
    int i = 0, j = 0;
    int count_real = 0;
    float k_real = 0, b_real = 0;
    short real_dot[2][20];

    if (!Flag_T_Right_Out)
    {
        return;
    }
    /************************实际左边线斜率***********************/
    for (i = 0; i < 2; i++)
    {
        for (j = 0; j < 20; j++)
        {
            real_dot[i][j] = 0;
        }
    }

    for (i = ImageH - 10; i >= EFFECTIVE_ROW; i--)
    {
        if (imageLine.Exist_Left[i])
        {
            real_dot[0][count_real] = (short)i;
            real_dot[1][count_real] = (short)imageLine.Point_Left[i];
            count_real++;
        }
        if (count_real == 20)
            break;
    }
    if (count_real == 20)
    {
        leastSquareMethod(real_dot[0], real_dot[1], 20, &k_real, &b_real);
    }
    if (count_real == 20 && k_real < -0.5 && k_real > -1.2)
    {
        Flag_T_Right_Lostline = 0;
    }
    if (count_real == 20 && k_real < -0.5 && k_real > -0.8)
    {
        Flag_T_Right_Out = 0;
        Flag_T_Right_OnlyOnce = 0;
        Flag_T_Right_Clc = 1;
    }
}
/***************************************************出入库************************************************************************/
void Garage_Out(void)
{
    int i = 0;
    int black_line = 0;

    if (Flag_Garage_Out == 1)
    {
        for (i = ImageH - 1; i > 0; i--)
        {
            if (imageLine.White_Num[i] == 0)
            {
                black_line++;
            }
        }
        if (black_line >= Ui_Garage_Out_Turn_Line)
        {
            Flag_Garage_Out = 0;
            Flag_Garage_Out_Turn = 1;
        }
    }
    if (Flag_Garage_Out_Turn)
    {
        if (Start_time == 0)
        {
            system_start();
            Start_time = system_getval_us();
        }
        End_time = system_getval_us();
        if (End_time - Start_time >= Ui_Garage_Out_Turn_Time * 1000000)
        {

            Start_time = 0;
            End_time = 0;
            Flag_Garage_Out_Turn = 0;
        }
    }
}
void Garage_In(void)
{
    int i = 0, j = 0;
    int JumpPoint_Cnt = 0;
    int Z_Line_Cnt = 0, Z_Line = 6;

    if (!Flag_Garage_In_Turn)
    {
        for (i = 59; i > Z_Line - 3; i--)
        {
            JumpPoint_Cnt = 0;
            for (j = 0; j < 94; j++)
            {
                if ((isWhite(i, j) && !isWhite(i, j - 1)) || (!isWhite(i, j) && isWhite(i, j - 1)))
                {
                    JumpPoint_Cnt++;
                }
            }
            if (JumpPoint_Cnt >= 14)
            {
                Z_Line_Cnt++;
                if (Z_Line == 6)
                {
                    Z_Line = i;
                }
            }
            if (Z_Line != 6 && JumpPoint_Cnt < 14)
            {
                return;
            }
        }
        if (Z_Line_Cnt >= 2 && Z_Line > 50)
        {
            Flag_Garage_In_Turn = 1;
        }
    }
    if (Flag_Garage_In_Turn)
    {
        if (Start_time == 0)
        {
            system_start();
            Start_time = system_getval_us();
            //开启计时器，计算打死时间
        }
        End_time = system_getval_us();
        if (End_time - Start_time >= Ui_Garage_In_Turn_Time * 1000000)
        {
            Start_time = 0;
            End_time = 0;
            Flag_Stop = 1;
            Flag_Garage_In_Turn = 0;
        }
    }
}
/***************************************************坡道************************************************************************/
void TF_Uart(void)//此函数放在串口中断中
{
    TF_dat[TF_num] = uart_read_byte(UART_1);//每获得一个数就先记到TF_uart_data中
    if (TF_num)
    {
        if (TF_dat[1] != 0x59)
        {
            TF_num = 0;                 //2、若第二个数不是 0x59，则TF_num清零，下一次重新检测第一个数
        }
        else  
            TF_num++;                   //3、第一和第二个数均满足要求，开始将获得的数，逐位赋给TF_dat[2~8]
    }
    if (TF_dat[0] != 0x59)
    {
        TF_num = 0;
    }
    else if (TF_num == 0) 
        TF_num = 1;                     //1、若第一个数满足要求0x59，则下一次接收并检测第二个函数

    if (TF_num == 9)//获得完9位数后
    {
        TF_num = 0;
        TF_checksum = TF_dat[0] + TF_dat[1] + TF_dat[2] + TF_dat[3] + TF_dat[4] + TF_dat[5] + TF_dat[6] + TF_dat[7];

        if ((TF_checksum % 256) == TF_dat[8])//TF_dat[8]是自检位，满足要求后根据公式计算距离等
        {
            TF_distance = TF_dat[3] << 8 | TF_dat[2];
            TF_strength = TF_dat[5] << 8 | TF_dat[4];
            TF_temperature = TF_dat[7] << 8 | TF_dat[6];
        }
    }
    char txt[16];
    sprintf(txt, "%d\n", TF_distance);
    uart_write_buffer(UART_1, &txt,16);
 
    if(TF_distance < 50 && TF_distance > 20)
    {
        Flag_Stop = 1;
    }
}
/**************************************************中线误差************************************************************************/
void Get_MidlineErr(void)
{
    int i;
    //int j;
    //int CenterPoint_Cnt = 0;

    //计算中线偏差
    for (i = Steer_pid.line; i < 60; i++)
    {
        if (imageLine1.Exist_Center[i])
        {
            Deviation = Constrain_Float((float)(imageLine1.Point_Center[i] - 45), -60, 60);
            break;
        }
    }
}