#ifndef LINE_EXTRACTION_ROS_H
#define LINE_EXTRACTION_ROS_H

#include <vector>
#include <string>
//#include <ros/ros.h>
#include "line_extraction.h"
#include "line.h"
#include "hokuyo.h"
#include "myself.h"

#define LINEEXTRACTIONDEBUG
namespace line_extraction
{

class LineExtractionHokuyo
{

public:
  // Constructor / destructor
  LineExtractionHokuyo();
  ~LineExtractionHokuyo();
  // Running
  void run();
  std::vector<Line> laserScanCallback(const hokuyo::LaserScan &);
private:

  ParameterReader pd_;
  LineExtraction line_extraction_;
  bool data_cached_; // true after first scan used to cache data
  // Members
  void loadParameters();
  void cacheData(const hokuyo::LaserScan &);
};

} // namespace line_extraction

#endif
