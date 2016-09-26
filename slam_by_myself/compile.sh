#!/bin/bash
rm -rf build
mkdir build
rm -rf lib
mkdir lib
rm -rf bin
mkdir bin
cd build
cmake ..
make -j8 -l8
cd ..
#./bin/laser_data_show /dev/ttyACM0
