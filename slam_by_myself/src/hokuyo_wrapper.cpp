#include "hokuyo_wrapper.h"

#ifdef _DEBUG_
 #include <iostream>
 using std::cout;
 using std::endl;
#endif

hokuyo_wrapper::hokuyo_wrapper( int argc, char** argv ) : information( argc, argv )
{
    runStatus = false;
}

bool hokuyo_wrapper::startHokuyo( )
{
    if ( runStatus )
        return true;

    // Open Hokuyo
    if ( !urg.open( information.device_or_ip_name( ),
                    information.baudrate_or_port_number( ),
                    information.connection_type( ) ) )
    {
#ifdef _DEBUG_
        cout << "Urg_driver::open( ): "
                     << information.device_or_ip_name( )
                     << ": " << urg.what( ) << endl;
#endif
        return false;
    }

    urg.set_scanning_parameter( urg.deg2step( -90 ), urg.deg2step( +90 ), 0 );

    // Normally, to receive measurement data from the sensor, the function start_measurement( )
    // and get_distance( ) are called (in pairs) many times( hand-shake measurement mode).
    // If the number of scans is used, the function start_measurement( ) is called just once,
    // and then only call get_distance(  ) repeatedly to receive data more effectively from the sensor.
    urg.start_measurement( qrk::Urg_driver::Distance, qrk::Urg_driver::Infinity_times, 0 );

    runStatus = true;

    return true;
}

void hokuyo_wrapper::setInterestRadius( long radius )
{
    interestRadius = radius;
}

bool hokuyo_wrapper::bufferDistance( )
{
    if ( runStatus )
    {
        long time_stamp = 0;
        if ( !urg.get_distance( distance_data, &time_stamp ) )
        {
#ifdef _DEBUG_
            cout << "Urg_driver::get_distance( ): " << urg.what( ) << endl;
#endif
            return false;
        }

        return true;
    }

    return false;
}

vector< long > hokuyo_wrapper::getDistance( ) const
{
    return distance_data;
}

void hokuyo_wrapper::showDistancePoints( cv::Mat& bkground )
{
    bkground.create( cv::Size( 480, 320 ), CV_8UC3 );

    const double maxScanDistance = urg.max_distance( );
    const double adjustable_factor = 7;
    const double scale = bkground.cols / 2 / maxScanDistance * adjustable_factor;

    // Draw axes
    cv::line( bkground,
              cv::Point( 10, bkground.rows * 3 / 4 ),
              cv::Point( bkground.cols - 10, bkground.rows * 3 / 4 ),
              cv::Scalar( 0, 255, 0 ), 2 );

    cv::line( bkground,
              cv::Point( bkground.cols / 2, 10 ),
              cv::Point( bkground.cols / 2, bkground.rows * 3 / 4 ),
              cv::Scalar( 0, 255, 0 ), 2 );
              
    for ( int index = 0; index < distance_data.size( ); index++ )
    {
        // Limit the distance data to interest radius
        if ( distance_data[index] > interestRadius )
        {
            distance_data[index] = interestRadius;
        }
        
        double x = distance_data[index] * cos( urg.index2rad( index ) );
        double y = distance_data[index] * sin( urg.index2rad( index ) );

        // Coordinate transformation for convenient display
        double tmpForSwap = x;
        x = y;
        y = tmpForSwap;
        x = -x * scale + bkground.cols / 2;
        y = -y * scale + bkground.rows * 3 / 4;

        cv::circle( bkground, cv::Point( x, y ), 1, cv::Scalar( 0, 0, 255 ) );
    }
}
