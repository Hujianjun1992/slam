#include "localgridgenerator.h"

#include <iostream>
#include <iomanip>
using std::cout;
using std::endl;
using std::setw;

#include <math.h>

LocalGridGenerator::LocalGridGenerator( )
{
   interestRadius = INTEREST_RADIUS;

   for ( int i = 0; i < GRID_HEIGHT; i++ )
   {
       for ( int j = 0; j < GRID_WIDTH; j++ )
       {
           grid[i][j].occupancyValue = 0;
           grid[i][j].isFree = true;
           grid[i][j].centralX = CELL_SIZE * ( GRID_HEIGHT - 0.5 - i );
           grid[i][j].centralY = CELL_SIZE * ( GRID_WIDTH / 2 - 0.5 - j );
       }
   }
}

void LocalGridGenerator::setInterestRadius( long radius )
{
   interestRadius = radius;
}

void LocalGridGenerator::updateGrid( vector< long > distance )
{
   const int occupancyValThreshold = 24;  // Wild value, to be tuned

   for ( int i = 0; i < GRID_HEIGHT; i++ )
   {
       for ( int j = 0; j < GRID_WIDTH; j++ )
       {
           for ( int index = 0; index < distance.size( ); index++ )
           {
               if ( distance[index] > interestRadius )
               {
                   continue;
               }

               double laserX = distance[index] * cos( index2rad( index ) );
               double laserY = distance[index] * sin( index2rad( index ) );

               if (    grid[i][j].centralX - CAR_LENGTH / 2 < laserX
                    && grid[i][j].centralX + CAR_LENGTH / 2 > laserX
                    && grid[i][j].centralY - CAR_WIDTH / 2 < laserY
                    && grid[i][j].centralY + CAR_WIDTH / 2 > laserY
                  )
               {
                   grid[i][j].occupancyValue++;
               }
           }

           if ( grid[i][j].occupancyValue > occupancyValThreshold )
               grid[i][j].isFree = false;
           else
               grid[i][j].isFree = true;
       }
   }
}

void LocalGridGenerator::showGrid( )
{
   cout << endl;
   cout << "Grid Info: " << endl;
   for ( int i = 0; i < GRID_HEIGHT; i++ )
   {
       for ( int j = 0; j < GRID_WIDTH; j++ )
       {
           cout << setw( 8 );
           /*
           cout << "("
                << grid[i][j].centralX / 1000
                << ","
                << grid[i][j].centralY / 1000
                << ","
                << grid[i][j].occupancyValue
                << ")";
           */
           cout << grid[i][j].occupancyValue;
       }
       cout << endl << endl;
   }
   cout << endl;
}

void LocalGridGenerator::showGridOccupancy( cv::Mat& bkground )
{
   bkground.create( cv::Size( 640, 480 ), CV_8UC3 );
   const double scale = 80;

   for ( int i = 0; i < GRID_HEIGHT; i++ )
   {
       for ( int j = 0; j < GRID_WIDTH; j++ )
       {
           //if ( grid[i][j].occupancyValue > occupancyValThreshold )
           if ( grid[i][j].isFree == false )
           {
               // Coordinate transformation for convenient display
               double x_display = grid[i][j].centralY / 1000;
               double y_display = grid[i][j].centralX / 1000;

               x_display = -x_display * scale + bkground.cols / 2;
               y_display = -y_display * scale + bkground.rows * 3 / 4;

               cv::circle( bkground, cv::Point( x_display, y_display ), 5, cv::Scalar( 0, 0, 255 ), 10 );

           }
           else
           {
               // Coordinate transformation for convenient display
               double x_display = grid[i][j].centralY / 1000;
               double y_display = grid[i][j].centralX / 1000;

               x_display = -x_display * scale + bkground.cols / 2;
               y_display = -y_display * scale + bkground.rows * 3 / 4;


               cv::circle( bkground, cv::Point( x_display, y_display ), 5, cv::Scalar( 0, 255, 0 ), 10 );

           }
       }
   }
}

int LocalGridGenerator::deg2step( double deg )
{
   return static_cast< int >( 4 * deg );
}

int LocalGridGenerator::step2index( int step )
{
   return step + 360;
}

double LocalGridGenerator::index2rad( int index )
{
   const double pi = 3.1415926;
   return ( index * pi / 720 - pi / 2 );
}

double LocalGridGenerator::deg2rad( double deg )
{
   const double pi = 3.1415926;
   return deg * pi / 180;
}

void LocalGridGenerator::clearGrid( )
{
   for ( int i = 0; i < GRID_HEIGHT; i++ )
   {
       for ( int j = 0; j < GRID_WIDTH; j++ )
       {
           grid[i][j].occupancyValue = 0;
       }
   }
}

void LocalGridGenerator::genWaypoint( double &x, double &y )
{
   // TODO:
   // How to search the best cell as a target?
}
