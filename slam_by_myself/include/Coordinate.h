#pragma once

#include <iostream>
#include "WeightedFit.h"

using namespace std;


static iPoint FieldPointA = ipoint(9192,0);
static iPoint FieldPointB = ipoint(0,9192);
static iPoint FieldPointC = ipoint(-9192,0);
static iPoint FieldPointD = ipoint(0,-9192);

static LinePara FieldLine1 = linePara(-1.0, 9192.3881554, FieldPointA, FieldPointB);
static LinePara FieldLine2 = linePara(1.0, 9192.3881554, FieldPointB, FieldPointC);
static LinePara FieldLine3 = linePara(-1.0, -9192.3881554, FieldPointC, FieldPointD);
static LinePara FieldLine4 = linePara(1.0, -9192.3881554, FieldPointD, FieldPointA);
static LinePara FieldLine5 = linePara(100000.0, 0.0, FieldPointB, FieldPointD);

static CirclePara FieldCircle1 = circlePara(-3000, 1301, 400, 350);
static CirclePara FieldCircle2 = circlePara(-1951, 880, 400, 350);
static CirclePara FieldCircle3 = circlePara(-651, 815, 400, 350);
static CirclePara FieldCircle4 = circlePara(-495, 2416, 400, 350);
static CirclePara FieldCircle5 = circlePara(-3347, -997, 400, 350);
static CirclePara FieldCircle6 = circlePara(-2400, -2848, 400, 350);
static CirclePara FieldCircle7 = circlePara(-1499, -2499, 400, 350);

static CirclePara FieldCircle8 = circlePara(3000, 1301, 400, 350);
static CirclePara FieldCircle9 = circlePara(1951, 880, 400, 350);
static CirclePara FieldCircle10 = circlePara(651, 815, 400, 350);
static CirclePara FieldCircle11 = circlePara(495, 2416, 400, 350);
static CirclePara FieldCircle12 = circlePara(3347, -997, 400, 350);
static CirclePara FieldCircle13 = circlePara(2400, -2848, 400, 350);
static CirclePara FieldCircle14 = circlePara(1499, -2499, 400, 350);


typedef struct
{
  int Tx;
  int Ty;
  float theta;
}CoorTransPara;

class Coordinate
{
 public:
  Coordinate(void);
  ~Coordinate(void);

  void CalCoorTransPara(CoorTransPara &transPara,
                        LinePara w1,
                        LinePara w2,
                        LinePara R1,
                        LinePara R2);
  void CoortransTest();
  void CalLaserCoord();

  CoorTransPara LaserCoordTransPara;
  void printLaserCoortransPara(CoorTransPara coordinate);
  void TransformCoord(CoorTransPara transPara, iPoint R, iPoint& W);
};
