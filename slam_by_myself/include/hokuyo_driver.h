# pragma once

#include <iostream>
#include <boost/function.hpp>
#include <boost/thread/thread.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "hokuyo.h"

using namespace std;
using namespace cv;

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
  void getLaserData(const hokuyo::LaserScan &scan);
  void DrawLaserData(const hokuyo::LaserScan &scan);

 private:
  typedef boost::function<void(const hokuyo::LaserScan &)> UseScanFunction;
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

  State state_;

  HokuyoConfig config_;

};
