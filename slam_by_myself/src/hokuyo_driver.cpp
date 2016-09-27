//#include <iostream>
//#include <math.h>
#include "hokuyo_driver.h"
#include "myself.h"

//using namespace std;

HokuyoDriver::HokuyoDriver(HokuyoConfig& config)
{
  config_ = config;
  // config_.min_ang = -1.7;
  // config_.max_ang = 1.7;
  // config_.intensity = false;
  // config_.cluster = 1;
  // config_.skip = 0;
  // config_.port = "/dev/ttyACM0";
  // config_.calibrate_time = true;
  // config_.time_offset = 0;
  // config_.allow_unsafe_settings = false;
  // config_.state = false;
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
    useScan_ = boost::bind(&HokuyoDriver::DrawLaserData, this, _1);
    useScan_(scan_);
  }
  try {
    laser_.stopScanning();
  } catch (hokuyo::Exception& e) {
    cout << "Exception thrown while trying to stop scan " << e.what() << "." << endl;
  }
  state_ = OPENED;

}

void
HokuyoDriver::getLaserData(const hokuyo::LaserScan &scan)
{
 //  for (int i = 0; i < scan.ranges.size(); ++i) {
 //    cout << scan.ranges[i];
 // }
 //  cout << endl;
}

void
HokuyoDriver::DrawLaserData(const hokuyo::LaserScan &scan)
{
   cout << scan.ranges.size() << endl;

   // Mat image(800, 800, CV_8UC3, Scalar(0,255,0));

   // imshow("laser_data",image);

}
