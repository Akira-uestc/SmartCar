#ifndef _LS_FUNCTION_H_
#define _LS_FUNCTION_H_

int Constrain_Int(int amt, int low, int high);
float Constrain_Float(float amt, float low, float high);
float getLineK(int x1, int y1, int x2, int y2);
float getLineValue(int x, float k, float b);
short my_abs_short(short in);
float my_abs_float(float in);

#endif
