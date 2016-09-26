cmake_minimum_required( VERSION 2.8 )
project( slam_by_myself )

set( CMAKE_CXX_STANDARD 11)
set( CMAKE_CXX_COMPILER "g++" )

set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )

include_directories( ${PROJECT_SOURCE_DIR}/include )
link_directories( ${PROJECT_SOURCE_DIR}/lib )

add_subdirectory( ${PROJECT_SOURCE_DIR}/src )
