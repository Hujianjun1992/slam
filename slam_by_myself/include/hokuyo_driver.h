# pragma once

#include <iostream>
#include <cmath>
#include <vector>
#include <boost/function.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "hokuyo.h"
#include "WeightedFit.h"
#include "clusering.h"
#include "myself.h"

#define MAX_DISTANCE 50
#define MIN_DISTANCE 0.5f
#define IMAGE_WIDTH 1500
#define IMAGE_HEIGHT 1500
using namespace std;
using namespace cv;

extern boost::mutex io_mutex;
extern Mat image_orgi;
extern Mat image_line;
extern Mat image_test;

static int usualColor[15] = {16777215,255,128,65280,32768,
                             16711680,16711935,8421376,65535,32896 };

/* typedef struct */
/* { */
/*   float x; */
/*   float y; */
/* }iPoint; */

/* typedef struct{ */
/*   double proportion; */
/*   double length; */
/*   double width; */
/*   iPoint Point1; */
/*   iPoint Point2; */
/*   iPoint Point3; */
/*   iPoint Point4; */
/* public: */
/*   Rectangle(double proportion_ = 0.2){proportion = proportion_;} */
/* }Rectangle; */

class Rectangle
{
 public:
  double proportion;
  double length;
  double width;
  vector<iPoint> Point;
 Rectangle(double proportion_ = 0.2):Point(4)
    {
    proportion = proportion_;
  }
};

struct HokuyoConfig
{
  double min_ang;
  double max_ang;
  bool intensity;
  int cluster;
  int skip;
  std::string port;
  bool calibrate_time;
  double time_offset;
  bool allow_unsafe_settings;

  bool state;

};


enum State{
  CLOSED = 0,
  OPENED = 1,
  RUNNING = 2
};

class HokuyoDriver
{
 public:

  HokuyoDriver(HokuyoConfig &config);
  ~HokuyoDriver();

  bool checkAngleRange(HokuyoConfig &config);
  bool checkIntensitySupport(HokuyoConfig &config);
  void doOpen();
  void doClose();
  void doStart();
  void doStop();
  std::string getID();
  void config_update(HokuyoConfig &new_config);
  void scanThread();
  void getLaserData();
  void DrawLaserData();
  void LaserDataCovert();

 private:
  typedef boost::function<void()> UseScanFunction;
  UseScanFunction useScan_;

  boost::shared_ptr<boost::thread> scan_thread_;

  std::string device_port_;
  std::string device_status_;
  std::string device_id_;
  std::string last_seen_device_id_;

  bool first_scan_;

  std::string vendor_name_;
  std::string product_name_;
  std::string protocol_version_;
  std::string firmware_version_;

  std::string connect_fail_;

  hokuyo::LaserScan  scan_;
  hokuyo::Laser laser_;
  hokuyo::LaserConfig laser_config_;

  bool calibrated_;
  int lost_scan_thread_count_;
  int corrupted_scan_count_;

  vector<float> LaserRho;
  vector<float> BreakedLaserRho;

  vector<float>LaserTheta;
  vector<float>BreakedLaserTheta;

  vector<float>SepLaserRho;
  vector<float>SepLaserTheta;  

  vector<int>LaserX;
  vector<int>LaserY;

  vector<int>BreakIndex;
  vector<LinePara>FittedLine;

  State state_;

  HokuyoConfig config_;

  void CreateLaserImage(Mat* LaserImage, vector<float>& LaserRho, vector<float>& LaserTheta);
  int BreakLadarRho();
  int PolyContourFit(float* X, float* Y, int n, float Eps);
  int BreakPolyLine();
  void FitLine(vector<LinePara>& FittedLine, vector<float>& LaserRho, vector<float>& LaserTheta);
  void DrawLaserLine(vector<LinePara>& FittedLine, Mat* LaserImage);
  void MedFilter(vector<float>& LaserRho, vector<float>& LaserTheta);
  Mat HoughLines(Mat* srcImage);

  vector<float> x_start;
  vector<float> x_end;
  vector<float> y_start;
  vector<float> y_end;
};
