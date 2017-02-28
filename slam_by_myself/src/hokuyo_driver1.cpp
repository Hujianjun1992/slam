#include "hokuyo_driver.h"
#include "myself.h"

//#define DEBUG

std::vector<int> CornerTemp;

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
    //useScan_ = boost::bind(&HokuyoDriver::DrawLaserData, this);
        useScan_ = boost::bind(&HokuyoDriver::getLaserData, this);
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
    static int ScanCount = 0;
    static std::ofstream *ff = new std::ofstream( "./data/LaserData.txt", std::ios::out );
    if ( firstCall == false )
    {
    *ff << "LaserConfing" << endl;
    *ff << "min_angle " << scan_.config.min_angle << endl;
    *ff << "max_angle " << scan_.config.max_angle << endl;
    *ff << "ang_increment " << scan_.config.ang_increment << endl;
    *ff << "time_increment " << scan_.config.time_increment << endl;
    *ff << "scan_time " << scan_.config.scan_time << endl;
    *ff << "min_range " << scan_.config.min_range << endl;
    *ff << "max_range " << scan_.config.max_range << endl;
    *ff << "range_res " << scan_.config.range_res << endl;
    firstCall = true;
   }
    else {
        ScanCount ++;
        *ff << "ScanCount " << ScanCount << endl;
        *ff << "self_time_stamp " << scan_.self_time_stamp << endl;
        *ff << "L " ;
        for ( int i = 0; i < scan_.ranges.size(); i ++ ) {
            if ( scan_.ranges[i] > 40.0  ) {
                scan_.ranges[i] = 40.0;
        }
            if ( scan_.ranges[i] < 0.15) {
                scan_.ranges[i] = 0.15;
            }
            *ff << scan_.ranges[i] << " ";
        }
        *ff << endl;

    }

    cout << " ScanCount    " << ScanCount << endl;

/////////////////////////////////////////////
////////////////////////////////////////////////

  Mat image_tmp_orgi(800, 800, CV_8UC3, Scalar(20, 20, 20));
  Mat image_tmp_line(800, 800, CV_8UC3, Scalar(20, 20, 20));
  Mat image_tmp_test(800, 800, CV_8UC3, Scalar(20, 20, 20));

  vector<line_extraction::Line> line_tmp;
  vector<float> SepLaserRho;
  vector<float> SepLaserTheta;
  line_tmp = lineextractionhokuyo.laserScanCallback(scan_);
  cout << BLUE"\t\t line num------------>" << RED << line_tmp.size() << RESET << endl;

  image_tmp_line = DrawLaserLine(line_tmp);

  LaserDataCovert();
  MedFilter(LaserRho, LaserTheta);
  BreakLadarRho();
  cout << "line cnt ----->" << BreakPolyLine(BreakedLaserRho, BreakedLaserTheta, SepLaserRho, SepLaserTheta) << endl;

  CreateLaserImage(&image_tmp_orgi, BreakedLaserRho, BreakedLaserTheta);

  io_mutex.lock();
  image_tmp_orgi.copyTo(image_orgi);
  image_tmp_test.copyTo(image_test);
  image_tmp_line.copyTo(image_line);
  io_mutex.unlock();
////////////////////////////////////////:
}

void
HokuyoDriver::DrawLaserData()
{
  Mat image_tmp_orgi(800, 800, CV_8UC3, Scalar(20, 20, 20));
  Mat image_tmp_line(800, 800, CV_8UC3, Scalar(20, 20, 20));
  Mat image_tmp_test(800, 800, CV_8UC3, Scalar(20, 20, 20));

  vector<line_extraction::Line> line_tmp;
  line_tmp = lineextractionhokuyo.laserScanCallback(scan_);
  cout << BLUE"\t\t line num------------>" << RED << line_tmp.size() << RESET << endl;

  image_tmp_line = DrawLaserLine(line_tmp);

  // for (uint i = 0; i < line_tmp.size(); ++i) {
  //   Vec4i l;
  //   boost::array<double, 2> start_tmp = line_tmp[i].getStart();
  //   boost::array<double, 2> end_tmp = line_tmp[i].getEnd();

  //   l[0] = start_tmp[0] * 50 + 400;
  //   l[1] = start_tmp[1] * 50 + 400;
  //   l[2] = end_tmp[0] * 50 + 400;
  //   l[3] = end_tmp[1] * 50 + 400;
  //   line ( image_tmp_line, Point(l[0],l[1]), Point(l[2],l[3]),Scalar(0, 0, 255), 1, CV_AA);
  // }

  LaserDataCovert();
  MedFilter(LaserRho, LaserTheta);
  CreateLaserImage(&image_tmp_orgi, LaserRho, LaserTheta);

  io_mutex.lock();
  image_tmp_orgi.copyTo(image_orgi);
  image_tmp_test.copyTo(image_test);
  image_tmp_line.copyTo(image_line);
  io_mutex.unlock();

  // cout << "line num----->" << line_tmp.size() << endl;

}
// void
// HokuyoDriver::DrawLaserData()
// {

//   Mat image_tmp_orgi(800, 800, CV_8UC3, Scalar(20, 20, 20));
//   Mat image_tmp_line(800, 800, CV_8UC3, Scalar(20, 20, 20));
//   Mat image_tmp_test(800, 800, CV_8UC3, Scalar(20, 20, 20));

//   LaserDataCovert();
//   MedFilter(LaserRho, LaserTheta);
//   // cout << RED"scan_.config.min_angle---->" RESET << scan_.config.min_angle << endl;
//   // cout << RED"scan_.config.max_angle---->" RESET << scan_.config.max_angle << endl;
//   // cout << RED"scan_.config.ang_increment---->" RESET << scan_.config.ang_increment << endl;
//   // cout << RED"scan_.config.time_increment---->" RESET << scan_.config.time_increment << endl;
//   // cout << RED"scan_.config.scan_time---->" RESET << scan_.config.scan_time << endl;
//   // cout << RED"scan_.config.min_range---->" RESET << scan_.config.min_range << endl;
//   // cout << RED"scan_.config.max_range---->" RESET << scan_.config.max_range << endl;
//   // cout << RED"scan_.config.range_res---->" RESET << scan_.config.range_res << endl;

//   //  cout << RED"breakCnt :" << RESET << BreakLadarRho() << endl ;

//   //  CreateLaserImage(&image_tmp_orgi, BreakedLaserRho, BreakedLaserTheta);


//   CreateLaserImage(&image_tmp_orgi, LaserRho, LaserTheta);

//   //  cout << YELLOW"lineCnt :" << RESET << BreakPolyLine() << endl;



//   image_tmp_line = HoughLines(&image_tmp_orgi);

//   //  FitLine(FittedLine, SepLaserRho, SepLaserTheta);
//   //  DrawLaserLine(FittedLine, &image_tmp_line);

//   io_mutex.lock();
//   image_tmp_orgi.copyTo(image_orgi);
//   image_tmp_test.copyTo(image_test);
//   image_tmp_line.copyTo(image_line);
//   io_mutex.unlock();


//   // cout << "\t" << RED << "show information " << RESET << endl;
//   // info_show("scan.config.min_angle",scan_.config.min_angle);
//   // info_show("scan.config.max_angle",scan_.config.max_angle);
//   // info_show("scan.config.ang_increment",scan_.config.ang_increment);
//   // info_show("scan.config.time_increment",scan_.config.time_increment);
//   // info_show("scan.config.scan_time",scan_.config.scan_time);
//   // info_show("scan.config.min_range",scan_.config.min_range);
//   // info_show("scan.config.max_range",scan_.config.max_range);
//   // info_show("scan.config.range_res",scan_.config.range_res);
//   // info_show("scan.self_time_stamp",scan_.self_time_stamp);
//   // info_show("scan.system_time_stamp",scan_.system_time_stamp);

// }

void
HokuyoDriver::CreateLaserImage(Mat* LaserImage, vector<float>& LaserRho, vector<float>& LaserTheta)
{
  //  zeros(LaserImage);
  //  *LaserImage = Mat::zeros()

  int dx = LaserImage->cols/2;
  int dy = LaserImage->rows/2;

  circle(*LaserImage, Point(dx,dy), 5, CV_RGB(0, 0, 255));

  int x,y;
  float theta,rho;
  //  float theta_offset = -M_PI/2;
  float theta_offset = 0.0;
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
        colorRGB = usualColor[colorIndex];
        R = colorRGB/65536;
        G = (colorRGB%65536)/256;
        B = colorRGB%256;
        colorIndex = (colorIndex + 1)%10;

        //for ( int i = 0; i < CornerTemp.size(); i ++ ) {

            //x = (int)(LaserRho[ CornerTemp[i] ]*cos(LaserTheta[ CornerTemp[i] ]+theta_offset)*len_shift) + dx;
            //y = (int)(LaserRho[ CornerTemp[i] ]*sin(LaserTheta[ CornerTemp[i] ]+theta_offset)*len_shift) + dy;

        //}
        x = (int)(LaserRho[i + 1]*cos(LaserTheta[i + 1]+theta_offset)*len_shift) + dx;
        y = (int)(LaserRho[i + 1]*sin(LaserTheta[i + 1]+theta_offset)*len_shift) + dy;

        cv::circle( *LaserImage, cv::Point(x, y), 3, CV_RGB(0, 255, 255) );
      }
    else
      {
        pointCnt ++;
      }

    x = (int)(rho*cos(theta+theta_offset)*len_shift) + dx;
    y = (int)(rho*sin(theta+theta_offset)*len_shift) + dy;

    //y = (int)(rho*cos(theta+theta_offset)*len_shift) + dx;
    //x = (int)(-rho*sin(theta+theta_offset)*len_shift) + dy;

    if (x >= 0 && x < LaserImage->cols && y >= 0 && y < LaserImage->rows) {
      pPixel = (unsigned char*)LaserImage->data + y*LaserImage->step + 3*x;
      pPixel[0] = B;
      pPixel[1] = G;
      pPixel[2] = R;
    }
    else{

    }
  }
/////////////////////////////////////////////////////////////
    float x1, y1;
    int dx1 = LaserImage->cols/2;
    int dy1 = LaserImage->rows/2;

    std::cout << "Corners.size " << Corners.size() << std::endl;
    for ( int i = 0; i < static_cast<int> (Corners.size()); i ++ ) {
        x1 = Corners[i].x;
        y1 = Corners[i].y;
        x1 = x1 * 50 + dx1;
        y1 = y1 * 50 + dy1;
        //cv::circle( *image, cv::Point(x,y), 3, CV_RGB(0, 255, 255), 3, 8, 0 );
        cv::circle( *LaserImage, cv::Point(x1,y1), 3, CV_RGB(0, 255, 255) );
    }

    ///////////////////////////////////////////////////////////////////



}

int
HokuyoDriver::BreakLadarRho()
{
  int breakCnt = 0;
  float rho = 0;

  float lastRho = LaserRho.at(0);
  float theta = LaserTheta.at(0);

  float dis = 0.0;
  float Dmax = .8;

  BreakedLaserRho.clear();
  BreakedLaserTheta.clear();

  BreakedLaserRho.push_back(lastRho);
  BreakedLaserTheta.push_back(theta);

  for (int i = 1; i < LaserRho.size(); i++) {
    rho = LaserRho.at(i);
    theta = LaserTheta.at(i);

    BreakedLaserRho.push_back(rho);
    BreakedLaserTheta.push_back(theta);
    dis = (float)abs(rho - lastRho);
    //   cout << RED"dis ------->" << dis  << RESET << endl;
    if (dis < Dmax) {

    }
    else
      {

        //        cout << GREEN"rho--->" << rho << "  " << "lastRho--->" << RESET << lastRho << endl;
        BreakedLaserRho.push_back(-1);
        BreakedLaserTheta.push_back(1000.0);
        breakCnt++;
      }
    lastRho = rho;
  }

  BreakedLaserRho.push_back(-1);
  BreakedLaserTheta.push_back(1000.0);
  return breakCnt;
}


int HokuyoDriver::PolyContourFit(float* X, float* Y, int n, float Eps)
{
    int cnt = 3;

  float dis = sqrt((float)(((X[0 + cnt]-X[n-1 - cnt])*(X[0 + cnt]-X[n-1 - cnt])) + ((Y[0 + cnt]-Y[n-1 - cnt])*(Y[0 + cnt]-Y[n-1 - cnt]))));
  float cosTheta = (X[n-1 - cnt] - X[0 + cnt])/dis;
  float sinTheta = -(Y[n-1 - cnt] - Y[0 + cnt])/dis;
  float MaxDis = 0;
  int i ;
  int MaxDisInd = -1;
  float dbDis;
  for (i = 1; i < n - 1; i++) {
    dbDis = abs( ( Y[i] - Y[0] ) * cosTheta + ( X[i] - X[0] ) * sinTheta );
    if (dbDis > MaxDis) {
      //      cout << "dbDis --------->" << dbDis << endl;
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
HokuyoDriver::BreakPolyLine(vector<float>& BreakedLaserRho, vector<float>& BreakedLaserTheta, vector<float>& SepLaserRho, vector<float>& SepLaserTheta)
{
  //  int data_num = scan_.ranges.size();
  //  cout << "data_num" << data_num << endl;
  //  cout << "scan_.ranges" << scan_.ranges.size() << endl;
///////////////////  int rho = 0;
///////////////////  float theta = 0.0;
///////////////////  float X[1200] = {0};
///////////////////  float Y[1200] = {0};
///////////////////  float rhoCopy[1200] = {0};
///////////////////  float thetaCopy[1200] = {0};
///////////////////  int pointCnt = 0;
///////////////////  int lineCnt = 0;
///////////////////  int N = 0;
///////////////////  SepLaserRho.clear();
///////////////////  SepLaserTheta.clear();
///////////////////
///////////////////  for (int i = 0; i < BreakedLaserRho.size(); ++i) {
///////////////////
///////////////////    rho = BreakedLaserRho.at(i);
///////////////////    theta = BreakedLaserTheta.at(i);
///////////////////
///////////////////    if (rho < 0) {
///////////////////      if (pointCnt > 250) {
///////////////////        N = PolyContourFit(X, Y, pointCnt, 0.3);
///////////////////
///////////////////        if (N == 0) {
///////////////////          lineCnt ++;
///////////////////
///////////////////          for (int j = 0; j < pointCnt; ++j) {
///////////////////            SepLaserRho.push_back(rhoCopy[j]);
///////////////////            SepLaserTheta.push_back(thetaCopy[j]);
///////////////////          }
///////////////////          SepLaserRho.push_back(-1);
///////////////////          SepLaserTheta.push_back(-1);
///////////////////        }
///////////////////        else if (N > 0) {
///////////////////          lineCnt +=2;
///////////////////
///////////////////          for (int j = 0 ; j < N; ++j) {
///////////////////            SepLaserRho.push_back(rhoCopy[j]);
///////////////////            SepLaserTheta.push_back(thetaCopy[j]);
///////////////////          }
///////////////////          SepLaserRho.push_back(-1);
///////////////////          SepLaserTheta.push_back(1000.0);
///////////////////
///////////////////          for (int j = N; j < pointCnt; ++j) {
///////////////////            SepLaserRho.push_back(rhoCopy[j]);
///////////////////            SepLaserTheta.push_back(thetaCopy[j]);
///////////////////          }
///////////////////          SepLaserRho.push_back(-1);
///////////////////          SepLaserTheta.push_back(1000.0);
///////////////////        }
///////////////////      }
///////////////////      pointCnt = 0;
///////////////////      continue;
///////////////////    }
///////////////////    X[pointCnt] = rho*cos(theta);
///////////////////    Y[pointCnt] = rho*sin(theta);
///////////////////    rhoCopy[pointCnt] = rho;
///////////////////    thetaCopy[pointCnt] = theta;
///////////////////    pointCnt ++;
///////////////////  }
///////////////////  return lineCnt;
int rho = 0;
  float theta = 0.0;
  float X[1200] = {0};
  float Y[1200] = {0};
  float rhoCopy[1200] = {0};
  float thetaCopy[1200] = {0};
  int pointCnt = 0;
  int lineCnt = 0;
  int N = 0;
  SepLaserRho.clear();
  SepLaserTheta.clear();

  vector<int> CornerIndex;
  int CornerCnt = 0;
  int tempIndex = 0;

    for ( int i = 0; i < BreakedLaserRho.size() ; i ++ ) {

        rho = BreakedLaserRho[i];
        theta = BreakedLaserTheta[i];

        if ( rho < 0 ) {
            if ( pointCnt > 200 ) {
                CornerIndex.clear();
                CornerCnt = FindCorners( CornerIndex, X, Y, 0, pointCnt, 0.5);

                if ( CornerIndex.size() == 0 ) {
                    for ( int k = 0; k < pointCnt; k ++ ) {
                        SepLaserRho.push_back( rhoCopy[k] );
                        SepLaserTheta.push_back( thetaCopy[k] );
                    }
                    SepLaserRho.push_back( -1 );
                    SepLaserTheta.push_back( 1000.0 );
                    lineCnt ++;
                } else {
                    tempIndex = 0;
                    for ( int k = 0; k < pointCnt; k ++ ) {
                        SepLaserRho.push_back( rhoCopy[k] );
                        SepLaserTheta.push_back( thetaCopy[k] );
                        if ( k == CornerIndex[tempIndex] ) {
                            SepLaserRho.push_back( -1 );
                            SepLaserTheta.push_back( 1000.0 );
                            lineCnt ++;
                            if ( tempIndex < static_cast<int>( CornerIndex.size() ) - 1 ) {
                                tempIndex ++;
                            }
                        }
                    }

                    SepLaserRho.push_back( -1 );
                    SepLaserTheta.push_back( 1000.0 );
                    lineCnt ++;
                }
            }
            //std::cout << "danteng!" << std::endl;
            pointCnt = 0;
            continue ;
        }
        X[ pointCnt ] = static_cast<double>( rho * cos(theta) );
        Y[ pointCnt ] = static_cast<double>( rho * sin(theta) );
        rhoCopy[ pointCnt ] = rho;
        thetaCopy[ pointCnt ] = theta;
        pointCnt++;
    }
return lineCnt;
}

int
HokuyoDriver::FindCorners( vector<int>& CornerIndex, float* X, float* Y, int start, int Cnt, float Eps ) {

    Corners.clear();
    int N = 0;
    int N1 = 0;
    int N2 = 0;

    N = PolyContourFit( X, Y, Cnt, Eps );
    std::cout << "NNNN------------>" << N << std::endl;

    if ( N == 0 ) {
        return 0;
    } else if ( N > 0 && N < Cnt ) {
        CornerTemp.push_back( start + N );
        CornerIndex.push_back( start + N );
        if ( N > 100 ) {
            N1 = FindCorners( CornerIndex, X, Y, start, N, Eps );
        }
        if ( Cnt - N > 100 ) {
            N2 = FindCorners( CornerIndex, X + N, Y + N, start + N, Cnt - N, Eps );
        }
    }
    int temp;
    for ( int i = 0; i < static_cast<int>( CornerIndex.size() ); i ++ ) {
        for ( int j = i + 1; j < static_cast<int>( CornerIndex.size() ); j ++ ) {
            if ( CornerIndex[i] > CornerIndex[j] ) {
                temp = CornerIndex[i];
                CornerIndex[i] = CornerIndex[j];
                CornerIndex[j] = temp;
            }
        }
    }
    iPoint CornerPoint;

    for ( int i = 0; i < static_cast<int>( CornerIndex.size() ); i ++ ) {
        CornerPoint.x = X[ CornerIndex[i] ];
        CornerPoint.y = Y[ CornerIndex[i] ];
        Corners.push_back( CornerPoint );
    }

    return CornerIndex.size();

}

void
HokuyoDriver::DrawLaserCorners( cv::Mat* image, vector<iPoint>& Corners ) {

    float x, y;
    int dx = image->cols/2;
    int dy = image->rows/2;

    std::cout << "Corners.size " << Corners.size() << std::endl;
    for ( int i = 0; i < static_cast<int> (Corners.size()); i ++ ) {
        x = Corners[i].x;
        y = Corners[i].y;
        x = x * 50 + dx;
        y = y * 50 + dy;
        //cv::circle( *image, cv::Point(x,y), 3, CV_RGB(0, 255, 255), 3, 8, 0 );
        cv::circle( *image, cv::Point(x,y), 3, CV_RGB(0, 255, 255) );
    }
}
void
HokuyoDriver::FitLine(vector<LinePara>& FittedLine, vector<float>& LaserRho, vector<float>& LaserTheta)
{
  float rho = 0;
  float theta = 0.0;
  float X[1200] = {0};
  float Y[1200] = {0};
  int pointCnt = 0;
  LinePara tmpLinePara;
  FittedLine.clear();
  // x_start.clear();
  // x_end.clear();
  // y_start.clear();
  // y_end.clear();
  for (int i = 0; i < LaserRho.size(); ++i) {
    rho = LaserRho.at(i);
    theta = LaserTheta.at(i);

    if (rho < 0) {
      // x_start.push_back(X[0]);
      // x_end.push_back(X[pointCnt]);
      // y_start.push_back(Y[0]);
      // y_end.push_back(Y[pointCnt]);
      //      cout << YELLOW"X------>" << X[0] << "\t" << Y[0] << RESET << endl;
      //      cout << RED"Y------>" << X[pointCnt] << "\t" << Y[pointCnt] << RESET << endl;
      WeightedFit(X, Y, pointCnt, &tmpLinePara);
      FittedLine.push_back(tmpLinePara);
      pointCnt = 0;
      continue;
    }

    X[pointCnt] = rho*cos(theta);
    //    X[pointCnt] = -rho*cos(theta-M_PI/2);
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
  //  cout << "FittedLine num :" << FittedLine.size() << endl;
}

Mat
HokuyoDriver::DrawLaserLine(vector<line_extraction::Line>& lines)
{


  Mat image_tmp(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3, Scalar(20, 20, 20));
  Vec4i l;
  boost::array<double, 2> start_tmp;
  boost::array<double, 2> end_tmp;
  int dx = image_tmp.cols/2;
  int dy = image_tmp.rows/2;
  int len_shift = 50;

  circle(image_tmp, Point(dx,dy), 5, CV_RGB(0, 0, 255));

  //  cout << "hujianjun" << lines.size() << endl;
  for (uint i = 0; i < lines.size(); ++i) {
    start_tmp = lines[i].getStart();
    end_tmp = lines[i].getEnd();
    l[0] = start_tmp[0] * len_shift + dx;
    l[1] = start_tmp[1] * len_shift + dy;
    l[2] = end_tmp[0] * len_shift + dx;
    l[3] = end_tmp[1] * len_shift + dy;
    //    cout << "hujianjun------>" <<  l[0] << "\t" << l[1] << "\t" << l[2] << "\t" << l[3] << endl;
    line ( image_tmp, Point(l[0],l[1]), Point(l[2],l[3]),Scalar(0, 0, 255), 1, CV_AA);
  }

  return image_tmp;
}
// void
// HokuyoDriver::DrawLaserLine(vector<LinePara>& FittedLine, Mat* LaserImage)
// {
//   int dx = LaserImage->cols/2;
//   int dy = LaserImage->rows/2;
//   circle(*LaserImage, Point(dx,dy), 5, CV_RGB(0, 0, 255));
//   float x1,y1,x2,y2;

//   int colorIndex = 0,colorRGB;
//   int R = 255, G = 0, B = 0;
//   for (int i = 0; i < FittedLine.size(); ++i) {
//     colorRGB = usualColor[colorIndex];
//     R = colorRGB/65536;
//     G = (colorRGB%65536)/256;
//     B = colorRGB%256;
//     colorIndex = (colorIndex + 1)%10;

//     x1 = FittedLine.at(i).startPoint.x;
//     y1 = FittedLine.at(i).startPoint.y;

//     x2 = FittedLine.at(i).endPoint.x;
//     y2 = FittedLine.at(i).endPoint.y;

//     x1 = x1 * 50 + dx;
//     y1 = y1 * 50 + dy;

//     x2 = x2 * 50 + dx;
//     y2 = y2 * 50 + dy;

//     //    cout << "hujianjun" << endl;

//     //    x1 = -x_start[i] * 50.0 + dx;
//     //    y1 = y_start[i] * 50.0 + dx;

//     //    x2 = -x_end[i] * 50.0 + dx;
//     //    y2 = y_end[i] * 50.0 + dy;
//     //    cout<<"x1: "<<x1<<" y1: "<<y1<<" x2: "<<x2<<" y2: "<<y2<<endl;
//     line(*LaserImage,Point(x2,y2),Point(x1,y1),CV_RGB(R,G,B),2,8,0);
//   }
//   //  for (int i = 0 ,i < x_start.size(), ++i)
//   //    {
//   //    }
// }

void
HokuyoDriver::LaserDataCovert()
{
  float theta = config_.min_ang;
  float deltaTeta = 0.25 * M_PI/180.0;

  LaserRho.clear();
  LaserTheta.clear();

  //  cout << "\t" << RED << "show information " << RESET << endl;

  for (int i = 0; i < scan_.ranges.size(); ++i) {
    if(scan_.ranges[i] > MAX_DISTANCE) scan_.ranges[i] = MAX_DISTANCE;
    if(scan_.ranges[i] < MIN_DISTANCE) scan_.ranges[i] = MIN_DISTANCE;
    LaserRho.push_back(scan_.ranges[i]);
    LaserTheta.push_back(theta);
    theta += deltaTeta;
    //    cout << i << "  :  " << LaserTheta[i] << "------->   " << LaserRho[i] << endl;

  }

}

void
HokuyoDriver::MedFilter(vector<float>& LaserRho, vector<float>& LaserTheta)
{
  //std::cout << 1 <<std::endl;
  vector<float> rho;
  vector<float> theta;

  //    rho.clear();
  //    theta.clear();

  //std::cout << 2 <<std::endl;
  int halfWindowSize = 2;
  float *neighbor = new float[2 * halfWindowSize + 1];
  float temp;

  //std::cout << 3 <<std::endl;

  for (int i = halfWindowSize; i < (int)LaserRho.size() - halfWindowSize; ++i) {
    for (int j = -halfWindowSize; j < halfWindowSize; j++) {
      neighbor[j + halfWindowSize] = LaserRho.at(i + j);
    }

    //std::cout << 4 <<std::endl;
    for (int m = 0; m < 2 * halfWindowSize + 1; ++m)
      {
        //std::cout << m <<std::endl;
        for (int n = m + 1; n < 2 * halfWindowSize + 1; ++n) {
          if (neighbor[m] > neighbor[n]) {
            temp = neighbor[m];
            neighbor[m] = neighbor[n];
            neighbor[n] = temp;
          }
        }
      }


    rho.push_back(neighbor[halfWindowSize]);
    theta.push_back(LaserTheta.at(i));
  }


  LaserRho.clear();
  LaserTheta.clear();

  for (int i = 0; i < (int)(rho.size()); ++i) {
    LaserRho.push_back(rho.at(i));
    LaserTheta.push_back(theta.at(i));
  }

}

Mat
HokuyoDriver::HoughLines(Mat* srcImage)
{

  int iter_num = 10;
  Mat midImage,dstImage;

  Canny(*srcImage, midImage, 3, 9, 3);
  cvtColor(midImage, dstImage, CV_GRAY2BGR);

  vector<Vec4i> lines;
  HoughLinesP(midImage, lines, 1, CV_PI/180, 50, 50, 20);

  cout << GREEN"line num : " << lines.size() << RESET << endl;

  //  vector<Rectangle> rect(lines.size());

  for (int i = 0; i < lines.size(); ++i) {
    Vec4i l = lines[i];
    //        double dis = pow( (l[3] - l[1]), 2) + pow( (l[2] - l[0]), 2);
        //        double length = pow(dis, 0.5);
        //        double width = length * 0.3;

    //    rectangle[i].width = rectangle[i].proportion * length;

        //p[0] = l[0]
    rectangle(dstImage, Point(l[0],l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 1, 8);

    //   line ( dstImage, Point(l[0],l[1]), Point(l[2],l[3]),Scalar(0, 0, 255), 1, CV_AA);
  }

#ifdef USING_CLUSER


  vector<vector<double> > linespara(lines.size());

  for (int i = 0; i < lines.size(); ++i){
    Vec4i l = lines[i];
    linespara[i].push_back((double)l[0]);
    linespara[i].push_back((double)l[1]);
    linespara[i].push_back((double)l[2]);
    linespara[i].push_back((double)l[3]);
  }
  vector<Cluster> cluster_line = k_means(linespara, 3, 100);

  for (uint i = 0; i < cluster_line.size(); ++i) {
    Vec4i p;
    for (uint j = 0; j < cluster_line[i].centroid.size(); ++j) {
      p[j] = cluster_line[i].centroid[j];
    }
    line ( dstImage, Point(p[0],p[1]), Point(p[2],p[3]),Scalar(0, 0, 255), 1, CV_AA);
  }


  vector<vector<Cluster> > clusters_out;
  for (int i = 0; i < iter_num; i++) {
      vector<Cluster> clusters = k_means(linespara, 4, 100);
      clusters_out.push_back(clusters);
  }

  for (uint i = 0; i < clusters_out.size(); ++i) {

    cout << "iteration : " << i << endl;
    vector<Cluster> cluster_tmp = clusters_out[i];

    for (uint j = 0; j < cluster_tmp.size(); ++j) {
      cout << "Cluster " << j << " :" << endl;

      cout << "\t" << "Centroid: " << "\n\t\t[ ";
      for (uint m = 0 ; m < cluster_tmp[j].centroid.size(); ++m) {
        cout << cluster_tmp[j].centroid[m] << " ";
      }
      cout << "]" << endl;

      cout << "\t" << "Samples:\n";
      for (uint n = 0; n < cluster_tmp[j].samples.size(); ++n) {
        uint c = cluster_tmp[j].samples[n];
        cout << "\t\t[ ";
        for (uint q = 0; q < linespara[0].size(); ++q) {
          cout << linespara[c][q] << " ";
        }
        cout << "]\n";
      }
    }
  }

#endif




  return dstImage;
}
