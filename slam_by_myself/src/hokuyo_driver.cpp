//#include <iostream>
//#include <math.h>
#include "hokuyo_driver.h"

#include "myself.h"

//using namespace std;

HokuyoDriver::HokuyoDriver(HokuyoConfig& config):config_(config)
{
}

HokuyoDriver::~HokuyoDriver()
{
}

bool
HokuyoDriver::checkAngleRange(HokuyoConfig &config)
{
  bool changed = false;
  if (config.min_ang < laser_config_.min_angle) {
    changed = true;
    config.min_ang = laser_config_.min_angle;
    cout << RED"config.min_ang < laser_config_.min_angle" RESET<< endl;
  }

  double max_safe_angular_range_per_cluster_deg = 95;
  if (firmware_version_ == "1.16.01(16/Nov./2009)") // 1.20.02(16/Jul./2013)
    max_safe_angular_range_per_cluster_deg = 190;

  int real_cluster = config.cluster == 0 ? 1 : config.cluster;
  double max_safe_angular_range = (real_cluster * max_safe_angular_range_per_cluster_deg) * M_PI / 180;

  if (config.intensity && (config.max_ang - config.min_ang) > max_safe_angular_range + 1e-8 && laser_.getProductName() == "SOKUIKI Sensor TOP-URG UTM-30LX") {
    changed = true;
    config.max_ang = config.min_ang + max_safe_angular_range;
  }

  if (config.max_ang - laser_config_.max_angle > 1e-10) {
    changed = true;
    config.max_ang = laser_config_.max_angle;
  }

  if (config.min_ang > config.max_ang) {
    changed = true;
    if (config.max_ang < laser_config_.min_angle) {
      if (laser_config_.min_angle - config.max_ang > 1e-10) {
        config.max_ang = laser_config_.min_angle;
      }
    }
    config.min_ang = config.max_ang;
  }

  return changed;
}

bool
HokuyoDriver::checkIntensitySupport(HokuyoConfig &config)
{
  if (config.intensity && !laser_.isIntensitySupported()) {
    cout << "This unit does not appear to support intensity mode. Turning intensity off." << endl;
    config.intensity = false;
    return true;
  }
  return false;
}

void
HokuyoDriver::doOpen()
{
  try {
    std::string old_device_id = device_id_;
    device_id_ = "unknown";
    device_status_ = "unknown";
    first_scan_ = true;

    laser_.open(config_.port.c_str());

    device_id_ = getID();
    vendor_name_ = laser_.getVendorName();
    firmware_version_ = laser_.getFirmwareVersion();
    product_name_ = laser_.getProductName();
    protocol_version_ = laser_.getProtocolVersion();

    device_status_ = laser_.getStatus();
    if (device_status_ != std::string("Sensor works well.")) {
      doClose();
      cout << "Sensor works not well." << endl;
      return ;
    }

    if(old_device_id != device_id_){
      cout << "Connected to device with ID: " << device_id_.c_str() << "." << endl;
      if (last_seen_device_id_ != device_id_) {
        last_seen_device_id_ = device_id_;
        calibrated_ = false;
      }

      for (int retries = 10; ; retries --) {
        try {
          laser_.laserOn();
          break;
        } catch (hokuyo::Exception&e) {
          if (!retries) {
            throw e;
          }
          else if (retries == 10) {
            cout << "Could not turn on laser. This may happen just after the device is plugged in. Will retry for 10 seconds." << endl;
          }
          sleep(1);
        }
      }

    }
    else {
      laser_.laserOn();
    }

    if (config_.calibrate_time && !calibrated_) {
      cout << "Starting calibration. This will take up a few seconds." << endl;
      double latency = laser_.calcLatency(false && config_.intensity, config_.min_ang, config_.max_ang, config_.cluster, config_.skip) * 1e-9;
      calibrated_ = true;
      cout << "Calibration finished. Latency is: " << latency << "." << endl;
    }
    else {
      calibrated_ = false;
      laser_.clearLatency();
    }

    cout << "Device opened successfully." << endl;
    laser_.getConfig(laser_config_);

    state_ = OPENED;
  } catch (hokuyo::Exception& e) {
    doClose();
    cout << "Exception thrown while opening Hokuyo " << e.what() << "." << endl;
    return;
  }

}

void
HokuyoDriver::doClose()
{
  try {
    laser_.close();
    cout << "Device closed successfully." << endl;
  } catch (hokuyo::Exception& e) {
    cout << "Exception thrown while trying to close: " << e.what() << endl;
  }
  state_ = CLOSED;
}

void
HokuyoDriver::doStart()
{
  try {
    laser_.laserOn();
    int status = laser_.requestScans(config_.intensity, config_.min_ang, config_.max_ang, config_.cluster, config_.skip);
    if (status != 0) {
      cout << "Failed to request scans from device. Status : " << status << "." << endl;
      corrupted_scan_count_++;
      return;
    }
    cout << "Waiting for first scan." << endl;
    state_ = RUNNING;
    scan_thread_.reset(new boost::thread(boost::bind(&HokuyoDriver::scanThread, this)));
  } catch (hokuyo::Exception& e) {
    doClose();
    cout << "Exception throw while starting Hokuyo " << e.what() << "." << endl;
    connect_fail_ = e.what();
    return;
  }

}

void
HokuyoDriver::doStop()
{
  if (state_ != RUNNING) {
    return;
  }
  state_ = OPENED;

  if (scan_thread_ && !scan_thread_->timed_join((boost::posix_time::milliseconds) 2000)) {
    cout << "scan_thread_ did not die after two seconds. Pretending that it did. This is probably a bad sign." << endl;
    lost_scan_thread_count_++;
  }
  scan_thread_.reset();

  cout << "Stopped." << endl;
}

std::string
HokuyoDriver::getID()
{
  std::string id = laser_.getID();
  if (id == std::string("H0000000")) {
    return "unknown";
  }
  return id;
}

void
HokuyoDriver::config_update(HokuyoConfig& new_config)
{
  cout << "Reconfigure called from state " << state_ << "." << endl;

  if (state_ == OPENED) {
    checkIntensitySupport(new_config);
    checkAngleRange(new_config);
  }
  config_ = new_config;
}

void
HokuyoDriver::scanThread()
{
  while (state_ == RUNNING) {
    try {
      int status = laser_.serviceScan(scan_);

      if (status != 0) {
        cout << "Error getting scan:" << status << "." << endl;
        break;
      }

    } catch (hokuyo::CorruptedDataException &e) {
      cout << "Skipping corrupted data ." << endl;
      continue;
    } catch (hokuyo::Exception& e) {
      cout << "Exception thrown while trying to get scan " << e.what() << "." << endl;
      doClose();
      return;
    }

    if (first_scan_) {
      first_scan_ = false;
      cout << "Streaming data." << endl;
    }
    useScan_ = boost::bind(&HokuyoDriver::DrawLaserData, this);
    //    useScan_ = boost::bind(&HokuyoDriver::getLaserData, this);
    useScan_();

  }
  try {
    laser_.stopScanning();
  } catch (hokuyo::Exception& e) {
    cout << "Exception thrown while trying to stop scan " << e.what() << "." << endl;
  }
  state_ = OPENED;

}

void
HokuyoDriver::getLaserData()
{
   for (int i = 0; i < scan_.ranges.size(); ++i) {
     cout << "scan.ranged[i] :" << scan_.ranges[i] << endl;
  }
   cout << "hujianjun" << endl;
}

void
HokuyoDriver::DrawLaserData()
{

  LaserDataCovert();

  Mat image_tmp(800, 800, CV_8UC3, Scalar(20, 20, 20));

  CreateLaserImage(&image_tmp);



  io_mutex.lock();
  image_tmp.copyTo(image);
  io_mutex.unlock();

  cout << "\t" << RED << "show information " << RESET << endl;
  info_show("scan.config.min_angle",scan_.config.min_angle);
  info_show("scan.config.max_angle",scan_.config.max_angle);
  info_show("scan.config.ang_increment",scan_.config.ang_increment);
  info_show("scan.config.time_increment",scan_.config.time_increment);
  info_show("scan.config.scan_time",scan_.config.scan_time);
  info_show("scan.config.min_range",scan_.config.min_range);
  info_show("scan.config.max_range",scan_.config.max_range);
  info_show("scan.config.range_res",scan_.config.range_res);
  info_show("scan.self_time_stamp",scan_.self_time_stamp);
  info_show("scan.system_time_stamp",scan_.system_time_stamp);

}

void
HokuyoDriver::CreateLaserImage(Mat* LaserImage)
{
  //  cvZero(LaserImage);

  int dx = LaserImage->cols/2;
  int dy = LaserImage->rows/2;

  circle(*LaserImage, Point(dx,dy), 5, CV_RGB(0, 0, 255));

  int x,y;
  float theta,rho;
  float theta_offset = -M_PI/2;
  int len_shift = 50;
  unsigned char * pPixel = 0;

  int colorIndex = 0,colorRGB;
  int R = 255,G = 0, B = 0;
  int pointCnt = 0;

  for (int i = 0; i < LaserRho.size(); ++i) {
    theta = LaserTheta.at(i);
    rho = LaserRho.at(i);

    if (rho < 0)
      {
        
      }
    x = -(int)(rho*cos(theta+theta_offset)*len_shift) + dx;
    y = (int)(rho*sin(theta+theta_offset)*len_shift) + dy;

    if (x >= 0 && x < LaserImage->cols && y >= 0 && y < LaserImage->rows) {
      pPixel = (unsigned char*)LaserImage->data + y*LaserImage->step + 3*x+2;
      *pPixel = 255;
    }
    else{

    }
  }

}

int
HokuyoDriver::BreakLadarRho()
{
  int breakCnt = 0;
  int rho = 0;

  int lastRho = LaserRho.at(0);
  float theta = LaserTheta.at(0);

  int dis = 0;
  float Dmax = 0.5;

  BreakedLaserRho.clear();
  BreakedLaserTheta.clear();

  BreakedLaserRho.push_back(lastRho);
  BreakedLaserTheta.push_back(theta);

  for (int i = 1; i < LaserRho.size(); i++) {
    rho = LaserRho.at(i);
    theta = LaserTheta.at(i);

    BreakedLaserRho.push_back(rho);
    BreakedLaserTheta.push_back(theta);
    dis = abs(rho - lastRho);
    if (dis < Dmax) {

    }
    else
      {
        BreakedLaserRho.push_back(-1);
        BreakedLaserTheta.push_back(1000.0);

        return breakCnt;
      }
  }
}


int HokuyoDriver::PolyContourFit(int* X, int* Y, int n, float Eps)
{

  float dis = sqrt((float)(((X[0]-X[n-1])*(X[0]-X[n-1])) + ((Y[0]-Y[n-1])*(Y[0]-Y[n-1]))));
  float cosTheta = (X[n-1] - X[0])/dis;
  float sinTheta = -(Y[n-1] - Y[0])/dis;
  float MaxDis = 0;
  int i ;
  int MaxDisInd = -1;
  float dbDis;
  for (i = 1; i < n - 1; i++) {
    dbDis = abs( ( Y[i] - Y[0] ) * cosTheta + ( X[i] - X[0] ) * sinTheta );
    if (dbDis > MaxDis) {
      MaxDis = dbDis;
      MaxDisInd = i;
    }
  }

  if (MaxDis > Eps) {
    return MaxDisInd;
  }

  return 0;
}

int
HokuyoDriver::BreakPolyLine()
{
  int rho = 0;
  float theta = 0.0;
  int X[1200] = {0};
  int Y[1200] = {0};
  int rhoCopy[1200] = {0};
  float thetaCopy[1200] = {0};
  int pointCnt = 0;
  int lineCnt = 0;
  int N = 0;
  SepLaserRho.clear();
  SepLaserTheta.clear();

  for (int i = 0; i < BreakedLaserRho.size(); ++i) {

    rho = BreakedLaserRho.at(i);
    theta = BreakedLaserTheta.at(i);

    if (rho < 0) {
      if (pointCnt > 200) {
        N = PolyContourFit(X, Y, pointCnt, 1000);

        if (N == 0) {
          lineCnt ++;

          for (int j = 0; j < pointCnt; ++j) {
            SepLaserRho.push_back(rhoCopy[j]);
            SepLaserTheta.push_back(thetaCopy[j]);
          }
          SepLaserRho.push_back(-1);
          SepLaserTheta.push_back(-1);
        }
        else if (N > 0) {
          lineCnt +=2;

          for (int j = 0 ; j < N; ++j) {
            SepLaserRho.push_back(rhoCopy[j]);
            SepLaserTheta.push_back(thetaCopy[j]);
          }
          SepLaserRho.push_back(-1);
          SepLaserTheta.push_back(1000.0);

          for (int j = N; j < pointCnt; ++j) {
            SepLaserRho.push_back(rhoCopy[j]);
            SepLaserTheta.push_back(thetaCopy[j]);
          }
          SepLaserRho.push_back(-1);
          SepLaserTheta.push_back(1000.0);
        }
      }
      pointCnt = 0;
      continue;
    }
    X[pointCnt] = rho*cos(theta);
    Y[pointCnt] = rho*sin(theta);
    rhoCopy[pointCnt] = rho;
    thetaCopy[pointCnt] = theta;
    pointCnt ++;
  }
  return lineCnt;
}

void
HokuyoDriver::FitLine(vector<LinePara>& FittedLine, vector<float>& LaserRho, vector<float>& LaserTheta)
{
  int rho = 0;
  float theta = 0.0;
  int X[1200] = {0};
  int Y[1200] = {0};
  int pointCnt = 0;
  LinePara tmpLinePara;
  FittedLine.clear();
  for (int i = 0; i < LaserRho.size(); ++i) {
    rho = LaserRho.at(i);
    theta = LaserTheta.at(i);

    if (rho < 0) {
      WeightedFit(X, Y, pointCnt, &tmpLinePara);
      FittedLine.push_back(tmpLinePara);
      pointCnt = 0;
      continue;
    }

    X[pointCnt] = rho*cos(theta);
    Y[pointCnt] = rho*sin(theta);
    pointCnt ++;
  }
  for (int i = 0; i < FittedLine.size(); ++i) {

    cout<<"a: "<<FittedLine.at(i).a<<"  b: "<<FittedLine.at(i).b<<" ";
    cout<<"x1: "<<FittedLine.at(i).startPoint.x<<" "
        <<"y1: "<<FittedLine.at(i).startPoint.y<<" "
        <<"x1: "<<FittedLine.at(i).endPoint.x<<" "
        <<"y1: "<<FittedLine.at(i).endPoint.y<<endl;
  }
}

void
HokuyoDriver::DrawLaserLine(vector<LinePara>& FittedLine, Mat* LaserImage)
{
  int dx = LaserImage->cols/2;
  int dy = LaserImage->rows/2;
  circle(*LaserImage, Point(dx,dy), 5, CV_RGB(0, 0, 255));
  float x1,y1,x2,y2;

  int colorIndex = 0,colorRGB;
  int R = 255, G = 0, B = 0;
  for (int i = 0; i < FittedLine.size(); ++i) {
    colorRGB = usualColor[colorIndex];
    R = colorRGB/65536;
    G = (colorRGB%65536)/256;
    B = colorRGB%256;
    colorIndex = (colorIndex + 1)%10;

    x1 = FittedLine.at(i).startPoint.x;
    y1 = FittedLine.at(i).startPoint.y;

    x2 = FittedLine.at(i).endPoint.x;
    y2 = FittedLine.at(i).endPoint.y;

    x1 = x1 * 50 + dx;
    y1 = -y1 * 50 + dy;

    x2 = x2 * 50 + dx;
    y2 = -y2 * 50 + dy;

    cout<<"x1: "<<x1<<" y1: "<<y1<<" x2: "<<x2<<" y2: "<<y2<<endl;
    line(*LaserImage,Point(x2,y2),Point(x1,y1),CV_RGB(R,G,B),2,8,0);
  }
}

void
HokuyoDriver::LaserDataCovert()
{
  float theta = config_.min_ang;
  float deltaTeta = 0.25 * M_PI/180.0;

  LaserRho.clear();
  LaserTheta.clear();

  //  cout << "\t" << RED << "show information " << RESET << endl;

  for (int i = 0; i < scan_.ranges.size(); ++i) {
    LaserRho.push_back(scan_.ranges[i]);
    LaserTheta.push_back(theta);
    theta += deltaTeta;
    //    cout << i << "  :  " << LaserTheta[i] << "------->   " << LaserRho[i] << endl;

  }

}
