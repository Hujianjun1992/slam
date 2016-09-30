#pragma once
#include <iostream>
#include <cmath>
#include<cv.h>
#include<highgui.h>
using namespace std;
#include <vector>
#include "WeightedFit.h"
#define UTM30LX

static int RadarImageWdith  = 720;
static int RadarImageHeight = 720;
//常用颜色
/*Colour      Red      Green      Blue      值     
    白色   White    255    255    255    16777215     
    红色    Red    255    0    0    255     
    深红色    Dark    Red    128    0    0    128     
    绿色    Green    0    255    0    65280     
    深绿色    Dark    Green    0    128    0    32768     
    蓝色    Blue    0    0    255    16711680       
    紫红色    Magenta    255    0    255    16711935            
    深紫    Dark    Cyan    0    128    128    8421376     
    黄色    Yellow    255    255    0    65535     
    棕色    Brown    128    128    0    32896  
 */
static int usualColor[15] = {16777215,255,128,65280,32768,
                             16711680,16711935,8421376,65535,32896 }; /*<10种常用的颜色*/

class OpenRadar
{
public:
    OpenRadar(void);
    ~OpenRadar(void);

    vector<int>RadarRho;
    vector<int>BreakedRadarRho;

    vector<double>RadarTheta;
    vector<double>BreakedRadarTheta;

    vector<int>SepRadarRho;//分段之后的Rho,折线也进行拆分
    vector<double>SepRadarTheta;

    vector<int>RadarX;
    vector<int>RadarY;

    vector<int>BreakIndex;
    vector<LinePara>FittedLine;

    //数据读取
    bool RadarRead(char *fileName);
    void CreateRadarImage(IplImage* RadarImage,vector<int>& RadarRho,vector<double>& RadarTheta);
    //数据分段
    int BreakRadarRho();//阈值如何选取
    int OpenRadar::PolyContourFit(int* X, int* Y, int n , float Eps);
    int BreakPolyLine();//将折线拆成两段
    void FitLine(vector<LinePara>& FittedLine,vector<int>& RadarRho,vector<double>& RadarTheta);//直线拟合
    void DrawRadarLine(vector<LinePara>& FittedLine,IplImage* RadarImage);
};

