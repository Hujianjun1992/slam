find_package(OpenCV REQUIRED)

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
find_package(Eigen3 REQUIRED)
include_directories( include ${Eigen3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIR})
find_package(Boost REQUIRED COMPONENTS
  system
  thread
  )
include_directories(SYSTEM ${Boost_INCLUDE_DIRS})


add_library(utilities utilities.cpp)
add_library(line line.cpp)
target_link_libraries(line utilities ${Boost_LIBRARIES})
add_library(line_extraction line_extraction.cpp)
target_link_libraries(line_extraction line)
add_library(line_extraction_hokuyo line_extraction_hokuyo.cpp)
target_link_libraries(line_extraction_hokuyo line line_extraction hokuyo)
#target_link_libraries(Urg_driver Tcpip Serial)
#add_library(hokuyo_wrapper hokuyo_wrapper.cpp)
#target_link_libraries(hokuyo_wrapper Urg_driver connect_utils Connection_information)
add_library(clusering clusering.cpp)
add_library(WeightedFit WeightedFit.cpp)
add_library(hokuyo hokuyo.cpp)
add_library(hokuyo_driver hokuyo_driver.cpp)
target_link_libraries(hokuyo_driver hokuyo WeightedFit  clusering line_extraction_hokuyo ${Boost_LIBRARIES} ${OpenCV_LIBS})



add_executable(laser_data_show laser_data_show.cpp)
target_link_libraries(laser_data_show hokuyo_driver ${Boost_LIBRARIES} ${OpenCV_LIBS})
