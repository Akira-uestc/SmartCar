#ifndef _CAMERA_H
#define _CAMERA_H

#define IMAGEH  120   /*!< 摄像头采集高度 */
#define IMAGEW  188   /*!< 摄像头采集宽度 */

/* 使用数组宽高 修改这里即可 */
#define LCDH    60  /*!< OLED显示高度（用户使用）高度 */
#define LCDW    94  /*!< OLED显示宽度（用户使用）宽度 */


//#define SMALLIMAGE  // 使用小尺寸显示60*94
#define MAX_ROW   LCDH
#define MAX_COL   LCDW

#include <zf_device_mt9v03x.h>

/** 图像原始数据存放 */
extern unsigned char Image_Data[IMAGEH][IMAGEW];

/** 压缩后之后用于存放屏幕显示数据  */
extern unsigned char Image_Use[LCDH][LCDW];

/** 二值化后用于OLED显示的数据 */
extern unsigned char Bin_Image[LCDH][LCDW];

void Get_Use_Image(void);
void Get_Bin_Image(unsigned char mode);
short GetOSTU(unsigned char tmImage[LCDH][LCDW]);
void lq_sobel(unsigned char imageIn[LCDH][LCDW], unsigned char imageOut[LCDH][LCDW], unsigned char Threshold);
void lq_sobelAutoThreshold(unsigned char imageIn[LCDH][LCDW], unsigned char imageOut[LCDH][LCDW]);
void Seek_Road(void);
void Bin_Image_Filter(void);
#endif
