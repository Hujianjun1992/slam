#include "hokuyo_driver.h"
#include <opencv2/opencv.hpp>
using namespace cv;

boost::mutex io_mutex;
Mat image(800, 800, CV_8UC3, Scalar(0, 255, 0));

int main(int argc, char *argv[])
{
  HokuyoConfig config;
  config.min_ang = -1.7;
  config.max_ang = 1.7;
  config.intensity = false;
  config.cluster = 1;
  config.skip = 0;
  config.port = "/dev/ttyACM0";
  config.calibrate_time = false;
  config.time_offset = 0;
  config.allow_unsafe_settings = false;
  config.state = true;

  HokuyoDriver hokuyodriver(config);

  hokuyodriver.doOpen();
  hokuyodriver.doStart();

  while(1)
    {
      io_mutex.lock();
      imshow("test", image);
      image = Mat::zeros(800, 800, CV_8UC3);
      io_mutex.unlock();
      if((char)cv::waitKey(33) == 'q')
        {
          break;
        }

    }

  hokuyodriver.doClose();
  return 0;
}
