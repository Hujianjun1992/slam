
#include"OpenRadar.h"
#include <iostream>
#include <cmath>
#include "io.h"

using namespace std;
const int MAX_POINT_COUNT = 1200;
int Rho[MAX_POINT_COUNT] = {0}; 


int main(){
    OpenRadar openRadar;
    char fileName[32] = "csv2\\data_0.csv";
    int frameCnt = 0;
    char key;
    IplImage* RadarImage = cvCreateImage(cvSize(RadarImageWdith,RadarImageHeight),IPL_DEPTH_8U,3);
    cvNamedWindow("Radar",1);
    cvNamedWindow("BreakedRadar",1);
    //cvNamedWindow("PloyLine",1);

    int lineCnt = 0;

    while (access(fileName,0) == 0)
    {
        sprintf(fileName,"csv2\\data_%d.csv",frameCnt);
        openRadar.RadarRead(fileName);  
        openRadar.CreateRadarImage(RadarImage,openRadar.RadarRho,openRadar.RadarTheta); 
        cvShowImage("Radar",RadarImage);

        
        openRadar.BreakRadarRho();
        openRadar.CreateRadarImage(RadarImage,openRadar.BreakedRadarRho,openRadar.BreakedRadarTheta);
        //cvShowImage("BreakedRadar",RadarImage);
        //在此处添加必要的算法

        //cvWaitKey(0);

        lineCnt = openRadar.BreakPolyLine();
        cout<<"lineCnt: "<<lineCnt<<endl;

        openRadar.CreateRadarImage(RadarImage,openRadar.SepRadarRho,openRadar.SepRadarTheta);
        cvShowImage("BreakedRadar",RadarImage);
        

        openRadar.FitLine(openRadar.FittedLine,openRadar.SepRadarRho,openRadar.SepRadarTheta);
        openRadar.DrawRadarLine(openRadar.FittedLine,RadarImage);
        cvShowImage("BreakedRadar",RadarImage);
        key = cvWaitKey(5);
           
        if (key == 27)//esc退出
        {
            break;
        }
        frameCnt++;
    }
    cvReleaseImage(&RadarImage);
    cvDestroyWindow("Radar");
    cvDestroyWindow("BreakedRadar");
    return 0;
}


