#include <iostream>
#include <unistd.h>
#include <boost/function.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include "hokuyo.h"
#include "myself.h"

#define PI 3.141592654f

using namespace std;

enum State{
  CLOSED = 0,
  OPENED = 1,
  RUNNING = 2
};

class OpenRadar
{
private:

  //  typedef boost::function<void(const hokuyo::LaserScan &)> UseScanFunction;
  //  UseScanFunction useScan_;
  boost::function<void(const hokuyo::LaserScan &)> useScan_;

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
  bool isintensitysupported;

  State state_;
public:
  OpenRadar():
    device_port_("/dev/ttyACM0")
  {

  }
  // virtual ~OpenRadar();

  bool checkIntensitySupport()
  {
    isintensitysupported = true;
    if (!laser_.isIntensitySupported()) {
      isintensitysupported = false;
      cout << "This unit does not appear to support intensity mode. Turning intensity off." << endl;
      return true;
    }
    return false;
  }

  void doOpen(){
    try {
      std::string old_device_id = device_id_;
      device_id_ = "unknown";
      device_status_ = "unknown";
      first_scan_ = true;

      laser_.open(device_port_.c_str());

      device_id_ = getID();
      vendor_name_ = laser_.getVendorName();
      firmware_version_ = laser_.getFirmwareVersion();
      product_name_ = laser_.getProductName();
      protocol_version_ = laser_.getProtocolVersion();

      device_status_ = laser_.getStatus();
      if (device_status_ != std::string("Sensor works well.")) {
        doClose();
        cout << "Laser returned abnormal status message, aborting: " << device_status_.c_str() << " You may be able to find further information at http://www.ros.org/wiki/hokuyo_node/Troubleshooting/" << endl;
        return ;
      }

      if (old_device_id != device_id_) {
        cout << "Connected to device with ID: " << device_id_.c_str() << "." << endl;
        if (last_seen_device_id_ != device_id_) {
          last_seen_device_id_ = device_id_;
          calibrated_ = false;
        }

        for (int retries = 10 ; retries < 0; retries--) {
          try{
            laser_.laserOn();
            break;
          }
          catch(hokuyo::Exception& e)
            {
              if (!retries)throw e;
              else if(retries == 10)
                cout << "Could not turn on laser. This may happen just after the device is plugged in. Will retry for 10 seconds." << endl;
              sleep(1);
            }
        }

      }
      else
        laser_.laserOn();



    } catch (hokuyo::Exception& e) {
      doClose();
      cout << "Exception thrown while opening Hokuyo.\n" << e.what() << endl;
      return;
    }

  }

  virtual std::string getID()
  {
    std::string id = laser_.getID();
    if (id == std::string("H0000000")) {
      return "unknown";
    }
    return id;
  }

  void doClose()
  {
    try {
      laser_.close();
      cout << "Device closed successfully." << endl;
    } catch (hokuyo::Exception& e) {
      cout << "Exception thrown while trying to close:\n" << e.what();
    }

    state_ = CLOSED;

  }

  void doStart()
  {
    try {
      laser_.laserOn();
      laser_config_.min_angle = -1.7;
      laser_config_.max_angle = 1.7;
      cout << isintensitysupported << endl;
      cout << laser_config_.min_angle << endl;
      cout << laser_config_.max_angle << endl;
      int status = laser_.requestScans(isintensitysupported,laser_config_.min_angle,laser_config_.max_angle);
      if (status != 0) {
        cout << "Failed to request scans from device.  Status: " << status << endl;
        corrupted_scan_count_++;
        return;
      }
      cout << "Waiting for first scan," << endl;
      state_ = RUNNING;
      scan_thread_.reset(new boost::thread(boost::bind(&OpenRadar::scanThread, this)));
    } catch (hokuyo::Exception& e) {
      //      cout << "hujianjun" << endl;
      cout << "Exception thrown while starting Hokuyo." << e.what();
      doClose();
      connect_fail_ = e.what();
      return;
    }
  }

  void doStop()
  {
    if(state_ != RUNNING)
      return;

    state_ = OPENED;

    if (scan_thread_ && !scan_thread_->timed_join((boost::posix_time::milliseconds) 2000)) {
      cout << "scan_thread_ did not die after two seconds. Pretending that it did. This is probably a bad sign." << endl;
      lost_scan_thread_count_++;
    }
    scan_thread_.reset();
    cout << "Stopped." << endl;
  }

  void scanThread()
   {
    while(state_ == RUNNING)
      {
        try {
          int status = laser_.serviceScan(scan_);

          if(status != 0)
            {
              cout << "Error getting scan: " << status << endl;
              break;
            }
        } catch (hokuyo::CorruptedDataException &e) {
          cout << "Skipping corrupted data" << endl;
          continue;
        } catch (hokuyo::Exception& e) {
          cout << "Exception thrown while trying to get scan" << e.what() << "." << endl;
          doClose();
          return;
        }

        if (first_scan_) {
          first_scan_ = false;
          cout << "Streaming data." << endl;
        }
        useScan_ = boost::bind(&OpenRadar::getlaserdata, this, _1);

        useScan_(scan_);
      }
    try {
      laser_.stopScanning();
    } catch (hokuyo::Exception& e) {
      cout << "Exception thrown while trying to stop scan " << e.what() << "." << endl;
    }
    state_ = OPENED;
   }

  void getlaserdata(const hokuyo::LaserScan &scan)
  {
    //    cout << "hujianjun" << endl;
    for (int i = 0; i < scan.ranges.size(); ++i) {
      cout << scan.ranges[i] ;
    }

    cout << "\t" << "laser_data" << endl;

  }
};


int main(int argc, char *argv[])
{

  OpenRadar openradar;
  openradar.doOpen();
  sleep(2);
  //  cout << openradar.checkIntensitySupport() << endl;
  openradar.doStart();

  while(1);

  openradar.doClose();
  // hokuyo::Laser laser;
  // hokuyo::LaserConfig config;
  // hokuyo::LaserScan scan;
  // // config.min_angle = 0.0; -2.35619
  // // config.max_angle = 0.0; 2.35619
  // // config.ang_increment = 0.0; 0.00436332
  // // config.time_increment = 0.0; 1.73611e-05
  // // config.scan_time = 0.0; 0.025
  // // config.min_range = 0.0; -2.35619
  // // config.max_range = 0.0; 2.35619
  // // config.range_res = 0.0; 4.56781e-41

  // if (argc!=2) {
  //   cout << RED"usage: getID /dev/ttyACM? [quiet]\nOutputs the device ID of a hokuyo at /dev/ttyACM?. Add a second argument for script friendly output." RESET << endl;
  //   return -1;
  // }

  // for (int retries = 10; retries; retries--) {
  //   try {
  //     laser.open(argv[1]);
  //     if(laser.portOpen())
  //       {
  //         break;
  //       }
  //   } catch (hokuyo::Exception &e) {
  //     cout << RED << e.what() << RESET << endl;
  //     laser.close();
  //   }
  //   if(retries < 1)
  //     return -1;
  //   sleep(1);
  // }
  //  laser.getConfig(config);
  // cout << "config.min_angle :" << config.min_angle << endl;
  // cout << "config.max_angle :" << config.max_angle << endl;
  // cout << "config.ang_increment :" << config.ang_increment << endl;
  // cout << "config.time_increment :" << config.time_increment << endl;
  // cout << "config.scan_time :" << config.scan_time << endl;
  // cout << "config.min_range :" << config.min_angle << endl;
  // cout << "config.max_angle :" << config.max_angle << endl;
  // cout << "config.range_res :" << config.range_res << endl;

  // cout << "Serial number of hokuyo :" << laser.getID() << endl;
  // cout << "Firmware version of hokuyo :" << laser.getFirmwareVersion() << endl;
  // cout << "Vendor name of hokuyo :" << laser.getVendorName() << endl;
  // cout << "Product name of hokuyo :" << laser.getProductName() << endl;
  // cout << "Protocol version of hokuyo :" << laser.getProtocolVersion() << endl;

  // //  int res = laser.pollScan(scan, -135.0*PI/180.0, 135.0*PI/180.0, 1, 1000);

  // int res = laser.laserReadline(buff,300,1000);
  // cout << "xuhong" << endl;


  // string ss;
  // ss = buff;
  // cout << res << endl;
  // cout << buff[200] << endl;
  // cout << ss << endl;
  // // if(res == 0)
  // //   {
  // //     cout << "hujianjun" << endl;
  // //     //   cout << scan.ranges.end()-scan.ranges.begin() << endl;
  // //   }
  // // else
  // //   {
  // //     //   cout << scan.ranges.end()-scan.ranges.begin() << endl;
  // //     cout << "error" << endl;
  // //   }

  return 0;
}
