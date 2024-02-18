#include <Function.h>

/*限幅函数*/
int Constrain_Int(int amt, int low, int high)
{
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}
/*浮点限幅函数*/
float Constrain_Float(float amt, float low, float high)
{
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}
/*求两点斜率*/
float getLineK(int x1, int y1, int x2, int y2)
{
    return ((float)y2 - (float)y1) / ((float)x2 - (float)x1);
}
/*kx+b*/
float getLineValue(int x, float k, float b)
{
    return (k * x + b);
}
/*求绝对值*/
short my_abs_short(short in)
{
    if (in < 0)
        return -in;
    else
        return in;
}
float my_abs_float(float in)
{
    if (in < 0.0f)
        return -in;
    else
        return in;
}
