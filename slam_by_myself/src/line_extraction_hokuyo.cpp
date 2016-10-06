#include "line_extraction_hokuyo.h"
#include <cmath>

namespace line_extraction
{

///////////////////////////////////////////////////////////////////////////////
// Constructor / destructor
///////////////////////////////////////////////////////////////////////////////
LineExtractionHokuyo::LineExtractionHokuyo():
  data_cached_(false),
  pd_("./cfg/line_extra_parameters.txt")
{
  loadParameters();

}

LineExtractionHokuyo::~LineExtractionHokuyo()
{
}

///////////////////////////////////////////////////////////////////////////////
// Run
///////////////////////////////////////////////////////////////////////////////
void LineExtractionHokuyo::run()
{
  // Extract the lines
  std::vector<Line> lines;
  line_extraction_.extractLines(lines);

}

///////////////////////////////////////////////////////////////////////////////
// Load ROS parameters
///////////////////////////////////////////////////////////////////////////////
void LineExtractionHokuyo::loadParameters()
{


  double bearing_std_dev, range_std_dev, least_sq_angle_thresh, least_sq_radius_thresh,
    max_line_gap, min_line_length, min_range, min_split_dist, outlier_dist;
  int min_line_points;

  bearing_std_dev = atof( pd_.getData( "bearing_std_dev" ).c_str() );
  line_extraction_.setBearingVariance(bearing_std_dev * bearing_std_dev);

  range_std_dev = atof( pd_.getData( "range_std_dev" ).c_str() );
  line_extraction_.setRangeVariance(range_std_dev * range_std_dev);

  least_sq_angle_thresh = atof( pd_.getData( "least_sq_angle_thresh" ).c_str() );
  line_extraction_.setLeastSqAngleThresh(least_sq_angle_thresh);

  least_sq_radius_thresh = atof( pd_.getData( "least_sq_radius_thresh" ).c_str() );
  line_extraction_.setLeastSqRadiusThresh(least_sq_radius_thresh);

  max_line_gap = atof( pd_.getData( "max_line_gap" ).c_str() );
  line_extraction_.setMaxLineGap(max_line_gap);

  min_line_length = atof( pd_.getData( "min_line_length" ).c_str() );
  line_extraction_.setMinLineLength(min_line_length);

  min_range = atof( pd_.getData( "min_range" ).c_str() );
  line_extraction_.setMinRange(min_range);

  min_split_dist = atof( pd_.getData( "min_split_dist" ).c_str() );
  line_extraction_.setMinSplitDist(min_split_dist);

  outlier_dist = atof( pd_.getData( "outlier_dist" ).c_str() );
  line_extraction_.setOutlierDist(outlier_dist);

  min_line_points = atoi( pd_.getData( "min_line_points" ).c_str() );
  line_extraction_.setMinLineLength(min_line_points);

#ifdef LINEEXTRACTIONDEBUG
  cout << RED"\t\t[line_extraction_parameters]\t\t" << RESET << endl;

  cout << BLUE"bearing_std_dev---->" << bearing_std_dev << RESET << endl;
  cout << BLUE"range_std_dev---->" << range_std_dev << RESET << endl;
  cout << BLUE"least_sq_angle_thresh---->" << least_sq_angle_thresh << RESET << endl;
  cout << BLUE"least_sq_radius_thresh---->" << least_sq_radius_thresh << RESET << endl;
  cout << BLUE"max_line_gap---->" << max_line_gap << RESET << endl;
  cout << BLUE"min_line_length---->" << min_line_length << RESET << endl;
  cout << BLUE"min_range---->" << min_range << RESET << endl;
  cout << BLUE"min_split_dist---->" << min_split_dist << RESET << endl;
  cout << BLUE"outlier_dist---->" << outlier_dist << RESET << endl;
  cout << BLUE"min_line_points---->" << min_line_points << RESET << endl;

  cout << RED"\t\t[line_extraction_parameters_show_end]\t\t" << RESET << endl;
#endif

}

///////////////////////////////////////////////////////////////////////////////
// Populate messages
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Cache data on first LaserScan message received
///////////////////////////////////////////////////////////////////////////////
void LineExtractionHokuyo::cacheData(const hokuyo::LaserScan &scan_msg)
{
  std::vector<double> bearings, cos_bearings, sin_bearings;
  std::vector<unsigned int> indices;
  unsigned int i = 0;
  for (double b = scan_msg.config.min_angle; b <= scan_msg.config.max_angle; b += scan_msg.config.ang_increment)
  {
    bearings.push_back(b);
    cos_bearings.push_back(cos(b));
    sin_bearings.push_back(sin(b));
    indices.push_back(i);
    ++i;
  }

  line_extraction_.setCachedData(bearings, cos_bearings, sin_bearings, indices);

}

///////////////////////////////////////////////////////////////////////////////
// Main LaserScan callback
///////////////////////////////////////////////////////////////////////////////
std::vector<Line> LineExtractionHokuyo::laserScanCallback(const hokuyo::LaserScan &scan_msg)
{
  std::vector<Line> lines;

  if (!data_cached_)
  {
    cout << "hujianjun" << endl;
    cacheData(scan_msg);
    data_cached_ = true;
  }
  std::vector<double> scan_ranges_doubles(scan_msg.ranges.begin(), scan_msg.ranges.end());
  line_extraction_.setRangeData(scan_ranges_doubles);


  line_extraction_.extractLines(lines);

  return lines;
}

} // namespace line_extraction

