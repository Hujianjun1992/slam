
#ifndef WEIGHTED_FIT_H
#define WEIGHTED_FIT_H
#include <cmath>
#include <cstdlib>
#include "QSort.h"

/* #define MAX_FITPOINTS_CNT 1000 */
/* #define K 5.0 */

typedef struct
{
    float x;
    float y;
}iPoint;
typedef struct{
    double a;//y = a*x + b
    double b;
    double Rho; // 该段直线的倾角
    iPoint startPoint;
    iPoint endPoint;
}LinePara;



int Med(int R[] , int Cnt);// 求取中值
int CalW(int X[] , int Y[] , int Cnt , LinePara * EstLinePara , int W[]);
int FitPara(int X[] , int Y[] , int Cnt ,LinePara * EstLinePara , int W[]);
int WeightedFit(int X[] , int Y[] , int Cnt , LinePara * EstLinePara);
#define cmp_pts( x, y )   ( x < y )    //  用于快速排序比较x < y , 得到的结果问哦升序排列
#endif
