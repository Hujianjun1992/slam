#include "directiongenerator.h"
#include <math.h>

void DirectionGenerator::setInterestRadius( long radius )
{
    interestRadius = radius;
}

int DirectionGenerator::deg2step( double deg )
{
    return static_cast< int >( 4 * deg );
}

int DirectionGenerator::step2index( int step )
{
    return step + 360;
}

double DirectionGenerator::index2rad( int index )
{
    const double pi = 3.1415926;
    return ( index * pi / 720 - pi / 2 );
}

double DirectionGenerator::deg2rad( double deg )
{
    const double pi = 3.1415926;
    return deg * pi / 180;
}

int DirectionGenerator::findIndexOfMaxValue( long sum[], int length )
{
    int indexOfMaxValue = 0;
    long maxVal = sum[0];

    for ( int i = 1; i < length; i++ )
    {
        if ( sum[i] > maxVal )
        {
            maxVal = sum[i];
            indexOfMaxValue = i;
        }
    }
    return indexOfMaxValue;
}

// This algorithm of selecting a deepest direction from fixed directions is not robust.
// And the computed resultant direction angle fluctuates severely.
double DirectionGenerator::selectFromFixedDirections( vector< long > distance )
{
    // Pre-define 9 samplers
    double angle1[ ] =
    {
        -90.0000,        -87.1875,        -84.3750,        -81.5625,        -78.7500,        -75.9375,        -73.1250,        -70.3125,        -67.5000
    };
    double angle2[ ] =
    {
        -67.5000,        -64.6875,        -61.8750,        -59.0625,        -56.2500,        -53.4375,        -50.6250,        -47.8125,        -45.0000
    };
    double angle3[ ] =
    {
        -45.0000,        -42.1875,        -39.3750,        -36.5625,        -33.7500,        -30.9375,        -28.1250,        -25.3125,        -22.5000
    };
    double angle4[ ] =
    {
        -22.5000,        -19.6875,        -16.8750,        -14.0625,        -11.2500,         -8.4375,         -5.6250,         -2.8125,               0
    };
    double angle5[ ] =
    {
        -11.2500,         -8.4375,         -5.6250,         -2.8125,               0,          2.8125,          5.6250,          8.4375,         11.2500
    };
    double angle6[ ] =
    {
              0,         2.8125,         5.6250,         8.4375,        11.2500,        14.0625,        16.8750,        19.6875,        22.5000
    };
    double angle7[ ] =
    {
        22.5000,        25.3125,        28.1250,        30.9375,        33.7500,        36.5625,        39.3750,        42.1875,        45.0000
    };
    double angle8[ ] =
    {
        45.0000,        47.8125,        50.6250,        53.4375,        56.2500,        59.0625,        61.8750,        64.6875,        67.5000
    };
    double angle9[ ] =
    {
        67.5000,        70.3125,        73.1250,        75.9375,        78.7500,        81.5625,        84.3750,        87.1875,        90.0000
    };

    double* angle[9] = { angle1, angle2, angle3, angle4, angle5, angle6, angle7, angle8, angle9 };

    long sum[9] = {0};
    int index[9][9] = {{0}};
    for ( int i = 0; i < 9; i++ )
    {
        for ( int j = 0; j < 9; j++ )
        {
            index[i][j] = step2index( deg2step( angle[i][j] ) );
            sum[i] += distance[index[i][j]];
            //cout << fixed << setprecision( 4 ) << setw(10) << angle[i][j];
        }
        //cout << endl << endl;
        sum[i] /= 9;
        cout << sum[i]/1000 << endl;
    }

    // Find the biggest value in array sum,
    // with its index, the direction angle is chosen.
    int directionIndex = findIndexOfMaxValue( sum, 9 );
    double directionAngle = 0;

    switch( directionIndex )
    {
    case 0:
        directionAngle = -78.75;
        break;
    case 1:
        directionAngle = -56.25;
        break;
    case 2:
        directionAngle = -33.75;
        break;
    case 3:
        directionAngle = -11.25;
        break;
    case 4:
        directionAngle = -0.0;
        break;
    case 5:
        directionAngle = 11.25;
        break;
    case 6:
        directionAngle = 33.75;
        break;
    case 7:
        directionAngle = 56.25;
        break;
    case 8:
        directionAngle = 78.75;
        break;
    default:
        directionAngle = 0.0;
    }

    return directionAngle;
}

double DirectionGenerator::sumArtificialForces( vector< long > distance )
{
    double sum_force_x = 0;
    double sum_force_y = 0;

    for ( unsigned int index = 0; index < distance.size( ); index++ )
    {
        double radian = index2rad( index );
        //cout << radian << endl;

        // Limit the distance data to interest radius
        if ( distance[index] > interestRadius )
        {
            distance[index] = interestRadius;
        }
#if 0
        // Suppose that, F = d
        artificial_force_seq[index].f_x = distance[ index ] * cos( radian );
        artificial_force_seq[index].f_y = distance[ index ] * sin( radian );
#endif

        // Suppose that, F = d * d
        artificial_force_seq[index].f_x = distance[ index ] * distance[ index ] * cos( radian );
        artificial_force_seq[index].f_y = distance[ index ] * distance[ index ] * sin( radian );

        sum_force_x += artificial_force_seq[index].f_x;
        sum_force_y += artificial_force_seq[index].f_y;
    }

    double cos_theta = sum_force_y / sqrt( ( SQUARE( sum_force_x ) + SQUARE( sum_force_y ) ) );
    double angle_in_radian = asin( cos_theta );
    double angle_in_deg = angle_in_radian * 180 / 3.14;

    return angle_in_deg;
}

double DirectionGenerator::sumFixedArtificialForces( vector< long > distance )
{
    // Pre-define 9 samplers
    double angle1[ ] =
    {
        -90.0000,        -87.1875,        -84.3750,        -81.5625,        -78.7500,        -75.9375,        -73.1250,        -70.3125,        -67.5000
    };
    double angle2[ ] =
    {
        -67.5000,        -64.6875,        -61.8750,        -59.0625,        -56.2500,        -53.4375,        -50.6250,        -47.8125,        -45.0000
    };
    double angle3[ ] =
    {
        -45.0000,        -42.1875,        -39.3750,        -36.5625,        -33.7500,        -30.9375,        -28.1250,        -25.3125,        -22.5000
    };
    double angle4[ ] =
    {
        -22.5000,        -19.6875,        -16.8750,        -14.0625,        -11.2500,         -8.4375,         -5.6250,         -2.8125,               0
    };
    double angle5[ ] =
    {
        -11.2500,         -8.4375,         -5.6250,         -2.8125,               0,          2.8125,          5.6250,          8.4375,         11.2500
    };
    double angle6[ ] =
    {
              0,         2.8125,         5.6250,         8.4375,        11.2500,        14.0625,        16.8750,        19.6875,        22.5000
    };
    double angle7[ ] =
    {
        22.5000,        25.3125,        28.1250,        30.9375,        33.7500,        36.5625,        39.3750,        42.1875,        45.0000
    };
    double angle8[ ] =
    {
        45.0000,        47.8125,        50.6250,        53.4375,        56.2500,        59.0625,        61.8750,        64.6875,        67.5000
    };
    double angle9[ ] =
    {
        67.5000,        70.3125,        73.1250,        75.9375,        78.7500,        81.5625,        84.3750,        87.1875,        90.0000
    };

    double* angle[9] = { angle1, angle2, angle3, angle4, angle5, angle6, angle7, angle8, angle9 };

    long sum[9] = {0};
    int index[9][9] = {{0}};
    for ( int i = 0; i < 9; i++ )
    {
        for ( int j = 0; j < 9; j++ )
        {
            index[i][j] = step2index( deg2step( angle[i][j] ) );
            sum[i] += distance[index[i][j]];
        }
        sum[i] /= 9;
    }

    // 9 forces are available here
    double sum_x = sum[0] * cos( deg2rad( -78.75 ) )
                 + sum[1] * cos( deg2rad( -56.25 ) )
                 + sum[2] * cos( deg2rad( -33.75 ) )
                 + sum[3] * cos( deg2rad( -11.25 ) )
                 + sum[4] * cos( deg2rad( 0 ) )
                 + sum[5] * cos( deg2rad( 11.25 ) )
                 + sum[6] * cos( deg2rad( 33.75 ) )
                 + sum[7] * cos( deg2rad( 56.25 ) )
                 + sum[8] * cos( deg2rad( 78.75 ) );

    double sum_y = sum[0] * sin( deg2rad( -78.75 ) )
                 + sum[1] * sin( deg2rad( -56.25 ) )
                 + sum[2] * sin( deg2rad( -33.75 ) )
                 + sum[3] * sin( deg2rad( -11.25 ) )
                 + sum[4] * sin( deg2rad( 0 ) )
                 + sum[5] * sin( deg2rad( 11.25 ) )
                 + sum[6] * sin( deg2rad( 33.75 ) )
                 + sum[7] * sin( deg2rad( 56.25 ) )
                 + sum[8] * sin( deg2rad( 78.75 ) );

    double cos_theta = sum_y / sqrt( ( SQUARE( sum_x ) + SQUARE( sum_y ) ) );
    double angle_in_radian = asin( cos_theta );
    double angle_in_deg = angle_in_radian * 180 / 3.14;

    return angle_in_deg;
}

double DirectionGenerator::slideWindow( vector< long > distance )
{
    long average[8] = {0};

    // Compute the average distance of these 9 segments
    for ( int i = 0; i < 8; i++ )
    {
        segAverage( average[i], distance, i * 90, i * 90 + 90 );
    }

    // 9 forces are available here
    double sum_x = average[0] * cos( deg2rad( -78.75 ) )
                 + average[1] * cos( deg2rad( -56.25 ) )
                 + average[2] * cos( deg2rad( -33.75 ) )
                 + average[3] * cos( deg2rad( -11.25 ) )
                 + average[4] * cos( deg2rad( 11.25 ) )
                 + average[5] * cos( deg2rad( 33.75 ) )
                 + average[6] * cos( deg2rad( 56.25 ) )
                 + average[7] * cos( deg2rad( 78.75 ) );

    double sum_y = average[0] * sin( deg2rad( -78.75 ) )
                 + average[1] * sin( deg2rad( -56.25 ) )
                 + average[2] * sin( deg2rad( -33.75 ) )
                 + average[3] * sin( deg2rad( -11.25 ) )
                 + average[4] * sin( deg2rad( 11.25 ) )
                 + average[5] * sin( deg2rad( 33.75 ) )
                 + average[6] * sin( deg2rad( 56.25 ) )
                 + average[7] * sin( deg2rad( 78.75 ) );

    double cos_theta = sum_y / sqrt( ( SQUARE( sum_x ) + SQUARE( sum_y ) ) );
    double angle_in_radian = asin( cos_theta );
    double angle_in_deg = angle_in_radian * 180 / 3.14;

    return angle_in_deg;
}

void DirectionGenerator::segAverage( long& sum, vector< long > distance, int start_indx, int end_indx )
{
    for ( int index = start_indx; index < end_indx; index++ )
    {
        // Limit the distance data to interest radius
        if ( distance[index] > interestRadius )
        {
            distance[index] = interestRadius;
        }
        sum += distance[index];
    }
    sum /= end_indx - start_indx;
}

double DirectionGenerator::process( vector< long > distance )
{
    return sumArtificialForces( distance );
    // Or
    //return selectFromFixedDirections( distance );
    // Or
    //return sumFixedArtificialForces( distance );
    // Or
    //return slideWindow( distance );
}

void DirectionGenerator::genWaypoint( vector< long > distance, double& x, double& y )
{
    double directionAngleDeg = process( distance );
    int waypointIndex = step2index( deg2step( directionAngleDeg ) );
    x = distance[ waypointIndex ] * cos( deg2rad( directionAngleDeg ) );
    y = distance[ waypointIndex ] * sin( deg2rad( directionAngleDeg ) );
}
