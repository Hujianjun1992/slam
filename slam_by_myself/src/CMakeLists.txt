#find_package(PCL REQUIRED COMPONENTS commom io visualization filters)
#
find_package(OpenCV REQUIRED)

#set(Boost_INCLUDE_DIR /usr/include/boost)
find_package(Boost REQUIRED COMPONENTS
  system
  thread
  )
#
#add_definitions(${Boost_DEFINITIONS})
include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
#link_libraries(${Boost_LIBRARY_DIRS})
#

#add_library(Connection_information Connection_information.cpp)
#add_library(Serial Serial.cpp)
#add_library(connection_utils connection_utils.cpp)
#add_library(Tcpip Tcpip.cpp)
#add_library(Urg_driver Urg_driver.cpp)
#target_link_libraries(Urg_driver Tcpip Serial)
#add_library(hokuyo_wrapper hokuyo_wrapper.cpp)
#target_link_libraries(hokuyo_wrapper Urg_driver connect_utils Connection_information)
add_library(WeightedFit WeightedFit.cpp)
add_library(hokuyo hokuyo.cpp)
add_library(hokuyo_driver hokuyo_driver.cpp)
target_link_libraries(hokuyo_driver hokuyo WeightedFit ${Boost_LIBRARIES} ${OpenCV_LIBS})



add_executable(laser_data_show laser_data_show.cpp)
target_link_libraries(laser_data_show hokuyo_driver ${Boost_LIBRARIES} ${OpenCV_LIBS})
