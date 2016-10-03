#include "hokuyo_driver.h"
#include <opencv2/opencv.hpp>
using namespace cv;

boost::mutex io_mutex;
Mat image_orgi(IMAGE_WIDTH, IMAGE_HEIGHT, CV_8UC3, Scalar(0, 255, 0));
Mat image_line(IMAGE_WIDTH, IMAGE_HEIGHT, CV_8UC3, Scalar(0, 255, 0));
Mat image_test(IMAGE_WIDTH, IMAGE_HEIGHT, CV_8UC3, Scalar(0, 255, 0));

int main(int argc, char *argv[])
{
  HokuyoConfig config;
  config.min_ang = -2.35;
  config.max_ang = 2.35;
  config.intensity = false;
  config.cluster = 1;
  config.skip = 0;
  config.port = "/dev/ttyACM0";
  config.calibrate_time = true;
  config.time_offset = 0;
  config.allow_unsafe_settings = false;
  config.state = true;

  HokuyoDriver hokuyodriver(config);

  hokuyodriver.doOpen();
  hokuyodriver.doStart();

  while(1)
    {
      io_mutex.lock();
      imshow("laser data show", image_orgi);
      imshow("laser line show", image_line);
      //      imshow("laser test show", image_test);
      io_mutex.unlock();
      if((char)cv::waitKey(33) == 'q')
        {
          break;
        }

    }

  hokuyodriver.doClose();
  return 0;
}

