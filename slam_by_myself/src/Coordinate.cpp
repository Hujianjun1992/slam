#include "Coordinate.h"
#define PI 3.141592654

Coordinate::Coordinate(void)
{

}

Coordinate::~Coordinate(void)
{

}

void Coordinate::CalCoorTransPara(CoorTransPara &transPara,
                                  LinePara w1,
                                  LinePara w2,
                                  LinePara R1,
                                  LinePara R2)
{
  float theta = (W1.Rho - R1.Rho + W2.Rho - R2.Rho)/2;

  float Xw = (float)(W1.b - W2.b)/(W2.a - W1.a);
  float Yw = W1.a * Xw + W1.b;

  float Xr = (float)(R1.b - R2.b)/(R2.a - R1.a);
  float Yr = R1.a * Xr + R1.b;

  iPoint crossPoint;
  iPoint vectorW1,vectorR1;

  if (W1.startPoint.x == W2.startPoint.x && W1.startPoint.y == W2.startPoint.y) {
    crossPoint = ipoint(W1.startPoint.x,W1.startPoint.y);
    vectorW1 = ipoint(W1.endPoint.x - W1.startPoint.x, W1.endPoint.y - W1.endPoint.y)
  }else if (W1.endPoint.x == W2.startPoint.x && W1.endPoint.y == W2.startPoint.y) {
    crossPoint = ipoint(W1.endPoint.x,W1.endPoint.y);
    vectorW1 = ipoint(W1.endPoint.x - W1.startPoint.x, W1.startPoint.y - W1.startPoint.y);
  }else if (W1.startPoint.x == W2.endPoint.x && W1.startPoint.y == W2.endPoint.y) {
    crossPoint = ipoint(W1.startPoint.x, W1.startPoint.y);
    vectorW1 = ipoint(W1.endPoint.x - W1.startPoint.x, W1.endPoint.y - W1.startPoint.y);
  }else if (W1.endPoint.x == W2.endPoint.x && W1.endPoint.y == W2.endPoint.y) {
    crossPoint = ipoint(W1.endPoint.x,W1.endPoint.y);
    vectorW1 = ipoint(W1.startPoint.x - W1.endPoint.x, W1.startPoint.y - W1.endPoint.y);
  }

  transPara.theta = theta;
  transPara.Tx = Tx;
  transPara.Ty = Ty;
  iPoint R1ToW;

  TransformCoord(transPara, R1, startPoint, R1ToW);

  vectorR1.z = R1ToW.x - crossPoint.x;
  vectorR1.y = R1ToW.y - crossPoint.y;

  if (vectorW1.x * vectorR1.x + vectorW1.y * vectorR1.y < 0) {
    transPara.theta = theta + PI;
    transPara.Tx = (int)(Xw - cos(transPara.theta) * Xr + sin(transPara.theta) * Yr);
    transPara.Ty = (int)(Yw - sin(transPara.theta) * Xy - cos(transPara.theta) * Yr);
  }else
    {

    }
}

void Coordinate::CoortransTest(){
  Coordinate coord;
  CoorTransPara coordtrans;
  coord.CalCoorTransPara(coordtrans, FieldLine1, FieldLine5, FieldLine2, FieldLine5);
  cout << "theta : " << coordtrans.theta * 180/PI << "Tx :" << coordtrans.Tx << "Ty :" << coordtrans.Ty << endl;
}

void Coordinate::printLaserCoortransPara(CoorTransPara coordtrans)
{
  cout << "theta : " << coordtrans.theta * 180/PI << "Tx : " << coordtrans.Tx << "Ty : " << coordtrans.Ty << endl;
}

void Coordinate::TransformCoord(CoorTransPara transPara, iPoint R, iPoint& W){
  W.x = (int)(R.x * cos(transPara.theta) - R.y * sin(transPara.theta));
  W.y = (int)(R.x * sin(transPara.theta) + R.y * cos(transPara.theta));
  W.x = W.x + transPara.Tx;
  W.y = W.y + transPara.Ty;
}
