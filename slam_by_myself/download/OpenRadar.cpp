#include "OpenRadar.h"
#define pi 3.141592653

OpenRadar::OpenRadar(void)
{
}
OpenRadar::~OpenRadar(void)
{
}
bool OpenRadar::RadarRead(char *fileName){
    FILE* fp = NULL;
    int rho = 0;
    double theta;
    double deltaTeta;
#ifdef UTM30LX
       theta = (135.0 + 90)*pi/180;
       deltaTeta = -0.25*pi/180;
#elif defined URG04LX
    theta = (120.0 + 90)*pi/180;
    deltaTeta = -0.36*pi/180;
#endif

    int totalCnt = 0;
    fp = fopen(fileName,"r");
    if (fp == NULL)
    {
        //cout<<"failed to read"<<endl;
        return false;
    }else {
        //cout<<"successed to read"<<endl;
        RadarRho.clear();
        RadarTheta.clear();

        while(!feof(fp))
        {

            fscanf(fp, "%d, ", &rho);
            RadarRho.push_back(rho);
            RadarTheta.push_back(theta);
            theta += deltaTeta;
            //printf("%d  ", rho);
        }
        //cout<<"Total Count: "<<RadarRho.size()<<endl;
        //
    }
    fclose(fp);
    return true;
}

void OpenRadar::CreateRadarImage(IplImage* RadarImage,vector<int>& RadarRho,vector<double>& RadarTheta){
    //RadarImage = cvCreateImage(cvSize(RadarImageWdith,RadarImageHeight),IPL_DEPTH_8U,1);
    cvZero(RadarImage);
    //在中心加上一个圆心
    //int halfWidth  = RadarImageWdith/2;
    //int halfHeight = RadarImageHeight/2;
    int dx =  RadarImageWdith/2;
    int dy =  RadarImageHeight*3/4;

    cvCircle(RadarImage, cvPoint(dx,dy),3, CV_RGB(0,255,255), -1, 8,0);

    int x,y;
    double theta,rho;
    unsigned char * pPixel = 0;
    


    //颜色
    int colorIndex = 0,colorRGB;
    int R = 255,G = 0,B = 0;
    int pointCnt = 0;
  

    for (int i = 0; i < RadarRho.size();i++)
    {
        //theta = (pointCnt/4.0 - 45)*pi/180;
        theta = RadarTheta.at(i);
        rho = RadarRho.at(i);
        if (rho < 0)
        {
           //雷达数据断点标志
            colorRGB = usualColor[colorIndex];
            R = colorRGB/65536;
            G = (colorRGB%65536)/256;
            B = colorRGB%256; 
            colorIndex = (colorIndex + 1)%10;
        }else {
            pointCnt++;
        }

        x = (int)(rho*cos(theta)/10) + dx;
        y = (int)(-rho*sin(theta)/10)+ dy;

        if (x >= 0 && x < RadarImageWdith && y >= 0 && y < RadarImageHeight)
        {
            pPixel = (unsigned char*)RadarImage->imageData + y*RadarImage->widthStep + 3*x;
            pPixel[0] = B;
            pPixel[1] = G;
            pPixel[2] = R;
        }else{
            //cout<<"x: "<<x<<"  y: "<<y<<endl;
        }
    }

}

int OpenRadar::BreakRadarRho(){
    int breakCnt = 0;
    int rho = 0;
    
    int lastRho = RadarRho.at(0);
    double theta = RadarTheta.at(0);
    int dis = 0;
    int Dmax = 50;

    BreakedRadarRho.clear();
    BreakedRadarTheta.clear();

    BreakedRadarRho.push_back(lastRho);
    BreakedRadarTheta.push_back(theta);

    for (int i = 1; i< RadarRho.size();i++)
    {
       rho = RadarRho.at(i);
       theta = RadarTheta.at(i);

       BreakedRadarRho.push_back(rho);
       BreakedRadarTheta.push_back(theta);
       dis = abs(rho - lastRho);
       if (dis < Dmax)
       {
          
       }else {
          BreakedRadarRho.push_back(-1);
          BreakedRadarTheta.push_back(1000.0);
          breakCnt++;
       }

       lastRho = rho;
    }
    BreakedRadarRho.push_back(-1);
    BreakedRadarTheta.push_back(1000.0);
    //cout<<"breakCnt: "<<breakCnt<<endl;
    return breakCnt;
}

// 进行多边形拟合： Points : 轮廓上的点      n -- 轮廓点数目  Eps -- 拟合精度
// 返回值： 若该轮廓段需要分段，则返回分段点在该轮廓点列中的索引，否则，返回 0 表示不需要分段
// 这里是整个算法计算复杂性最大的一个地方
// 为了提高程序运行效率，对点到直线的距离计算进行改进：
// 多边形拟合中的直线是由点列中的点决定的
// 为了计算点到直线的距离，
// 采用坐标系旋转，将直线旋转到x轴方向，这样点到直线的距离即为各个点
// 在坐标旋转后的y值的绝对值
// 同时，坐标旋转矩阵在该次运算中为定值，只需一次计算，不需要多次的开方或三角计算
int OpenRadar::PolyContourFit( int* X, int* Y, int n , float Eps ) // 根据轮廓点，用多边形拟合该轮廓点    
{
    double dis = sqrt((double)(((X[0] - X[n - 1])*(X[0] - X[n - 1])) +  
                     ((Y[0] - Y[n - 1])* (Y[0] - Y[n - 1]))));
    double cosTheta = (X[n- 1] - X[0]) / dis;
    double sinTheta = - ( Y[n- 1] - Y[0] )/dis;
    double MaxDis = 0;
    int i ;
    int MaxDisInd = -1;
    double dbDis;
    for(i = 1 ; i < n - 1 ; i++)
    {
        // 进行坐标旋转，求旋转后的点到x轴的距离
        dbDis = abs( (Y[i] - Y[0]) * cosTheta + (X[i] - X[0])* sinTheta);
        if( dbDis > MaxDis)
        {
            MaxDis = dbDis;
            MaxDisInd = i;
        }
    }
    if(MaxDis > Eps)
    {
        return MaxDisInd;
        //        cout << "Line 1 : " << endl;
        //        cout << "Start :" << Points[0].x << "  " << Points[0].y  << " --- " << Points[MaxDisInd].x << "  " << Points[MaxDisInd].y << endl;
        //        cout << "角度： "<<180 * atan2(Points[0].y - Points[MaxDisInd].y , Points[0].x - Points[MaxDisInd].x ) / 3.1415926;
        //        cout << "Line 2 :" << endl;
        //        cout << "Start :" << Points[MaxDisInd].x << "  " << Points[MaxDisInd].y  << " --- " << Points[n - 1].x << "  " << Points[n - 1].y << endl;
        //        cout << "角度： "<< 180 * atan2(Points[n - 1].y - Points[MaxDisInd].y , Points[n - 1].x - Points[MaxDisInd].x ) / 3.1415926;
    }
    //    else{
    //        cout << "Line 1 : " << endl;
    //        cout << "Start :" << Points[0].x << "  " << Points[0].y  << " --- " << Points[n - 1].x << "  " << Points[n - 1].y << endl;
    //        cout << "角度： "<<180 * atan2(Points[n - 1].y - Points[0].y , Points[n - 1].x - Points[0].x ) / 3.1415926;

    //    }
    return 0;
}

//将折线拆成两段
int OpenRadar::BreakPolyLine(){
    int rho = 0;
    double theta = 0.0;
    int X[1200] = {0};
    int Y[1200] = {0};
    int rhoCopy[1200] = {0};
    double thetaCopy[1200] = {0};
    int pointCnt = 0;
    int lineCnt = 0;
    int N = 0;
    SepRadarRho.clear();
    SepRadarTheta.clear();

    for (int i = 0; i < BreakedRadarRho.size();i++)
    {
        rho = BreakedRadarRho.at(i);
        theta = BreakedRadarTheta.at(i);

        if (rho < 0)
        {
            if (pointCnt > 200)//数目比较少的点直接抛弃
            {
                N = PolyContourFit(X , Y, pointCnt, 1000);
                //cout<<"N: "<<N<<endl;
                //cout<<"pointCnt: "<<pointCnt<<endl;
                //如果不是直线，需要添加新的断点
                if (N == 0 )
                {
                    lineCnt++;
                    //cout<<"Line"<<endl;
                    for (int j = 0; j < pointCnt;j++)
                    {
                       
                       SepRadarRho.push_back(rhoCopy[j]);
                       SepRadarTheta.push_back(thetaCopy[j]);
                    }
                    SepRadarRho.push_back(-1);
                    SepRadarTheta.push_back(1000.0);

                }else if (N > 0)
                {
                    lineCnt += 2;

                    for (int j = 0; j < N;j++)
                    {
                        SepRadarRho.push_back(rhoCopy[j]);
                        SepRadarTheta.push_back(thetaCopy[j]);
                    }
                    SepRadarRho.push_back(-1);
                    SepRadarTheta.push_back(1000.0);

                    for (int j = N; j< pointCnt;j++)
                    {
                        SepRadarRho.push_back(rhoCopy[j]);
                        SepRadarTheta.push_back(thetaCopy[j]);
                    }
                    SepRadarRho.push_back(-1);
                    SepRadarTheta.push_back(1000.0);
                }   
            }
            pointCnt = 0;
            continue;
        }
        X[pointCnt] = rho*cos(theta);
        Y[pointCnt] = rho*sin(theta);
        rhoCopy[pointCnt]   = rho;
        thetaCopy[pointCnt] = theta;
        pointCnt++;
    }
    //cout<<"lineCnt: "<<lineCnt<<endl;
    return lineCnt;
}


void OpenRadar::FitLine(vector<LinePara>& FittedLine,vector<int>& RadarRho,vector<double>& RadarTheta){

    int rho = 0;
    double theta = 0.0;
    int X[1200] = {0};
    int Y[1200] = {0};
    int pointCnt = 0;
    LinePara tmpLinePara;
    FittedLine.clear();
    for (int i = 0 ; i < RadarRho.size();i++)
    {
        rho = RadarRho.at(i);
        theta = RadarTheta.at(i);

        if (rho < 0)
        {
            WeightedFit(X ,Y ,pointCnt,&tmpLinePara);
            FittedLine.push_back(tmpLinePara);
            pointCnt = 0;
            continue;
        }

        X[pointCnt] = rho*cos(theta);
        Y[pointCnt] = rho*sin(theta);
        pointCnt++;
    }
    for (int i = 0; i < FittedLine.size();i++)
    {
        cout<<"a: "<<FittedLine.at(i).a<<"  b: "<<FittedLine.at(i).b<<" ";
        cout<<"x1: "<<FittedLine.at(i).startPoint.x<<" "
            <<"y1: "<<FittedLine.at(i).startPoint.y<<" "
            <<"x1: "<<FittedLine.at(i).endPoint.x<<" "
            <<"y1: "<<FittedLine.at(i).endPoint.y<<endl;
    }

}

void OpenRadar::DrawRadarLine(vector<LinePara>& FittedLine,IplImage* RadarImage){
    //在中心加上一个圆心
    int dx =  RadarImageWdith/2;
    int dy =  RadarImageHeight*3/4;
    cvCircle(RadarImage, cvPoint(dx,dy),3, CV_RGB(0,255,255), -1, 8,0);
    int x1,y1,x2,y2;
    //颜色
    int colorIndex = 0,colorRGB;
    int R = 255,G = 0,B = 0;
    for (int i = 0; i < FittedLine.size();i++)
    {
        //雷达数据断点标志
        colorRGB = usualColor[colorIndex];
        R = colorRGB/65536;
        G = (colorRGB%65536)/256;
        B = colorRGB%256; 
        colorIndex = (colorIndex + 1)%10;

        x1 = FittedLine.at(i).startPoint.x;
        y1 = FittedLine.at(i).startPoint.y;

        x2 = FittedLine.at(i).endPoint.x;
        y2 = FittedLine.at(i).endPoint.y;

        //转化为Radar图坐标系下
        x1 = x1/10  + dx;
        y1 = -y1/10 + dy;
        x2 = x2/10  + dx;
        y2 = -y2/10 + dy;
        cout<<"x1: "<<x1<<" y1: "<<y1<<" x2: "<<x2<<" y2: "<<y2<<endl;
        cvLine(RadarImage,cvPoint(x2,y2),cvPoint(x1,y1),CV_RGB(R,G,B),2,8,0);
    }
}