#include "hokuyo_driver.h"
#include <opencv2/opencv.hpp>
using namespace cv;

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
      Mat image(800, 800, CV_8UC3, Scalar(0, 255, 0));
      imshow("test", image);
      if(waitKey(10) == 27)
        {
        cout << "test" << endl;
        break;
        }
    }

  hokuyodriver.doClose();
  return 0;
}

