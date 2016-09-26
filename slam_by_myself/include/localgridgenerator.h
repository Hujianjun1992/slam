#ifndef LOCALGRIDGENERATOR_H
#define LOCALGRIDGENERATOR_H

#include <vector>
using std::vector;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define CAR_WIDTH 600
#define CAR_LENGTH 750
#define GRID_WIDTH 14
#define GRID_HEIGHT 12
#define CELL_SIZE 300
#define INTEREST_RADIUS 14*CELL_SIZE

struct grid_cell
{
    double centralX;
    double centralY;
    int occupancyValue;
    bool isFree;
};

class LocalGridGenerator
{
public:
    LocalGridGenerator( );
    void setInterestRadius( long radius = INTEREST_RADIUS );
    void updateGrid( vector< long > distance );
    void genWaypoint( double& x, double& y );
    void clearGrid( );
    void showGrid( );
    void showGridOccupancy( cv::Mat& bkground );
private:
    long interestRadius;
    grid_cell grid[GRID_HEIGHT][GRID_WIDTH];

    // Mapping: [-90 deg, +90 deg] -> [-360, +360] -> [0, 720]
    int deg2step( double deg );
    int step2index( int step );
    double index2rad( int index );
    double deg2rad( double deg );
};

#endif // LOCALGRIDGENERATOR_H
