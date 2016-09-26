#ifndef HOKUYO_WRAPPER_H
#define HOKUYO_WRAPPER_H

// System Includes
#include <vector>
using std::vector;

// OpenCV Includes, in order to provide a simple gui window
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Urg Library Includes
#include <Urg_driver.h>
#include <math_utilities.h>
#include "Connection_information.h"

using namespace qrk;

class hokuyo_wrapper
{
public:
    hokuyo_wrapper( int argc, char** argv );
    bool startHokuyo( );
    // Set interest scanning radius, the unit is mm
    void setInterestRadius( long radius );
    bool bufferDistance( );
    vector< long > getDistance( ) const;
    void showDistancePoints( cv::Mat& bkground );
private:
    Connection_information information;
    long interestRadius;
public:
    Urg_driver urg;
private:
    bool runStatus;

    vector< long > distance_data;
};

#endif // HOKUYO_WRAPPER_H

