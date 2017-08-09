cmake_minimum_required(VERSION 2.8.3)
project(drive_and_map)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)
find_package(Eigen3)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
  DEPENDS Eigen
)

include_directories(include ${catkin_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/src ${Eigen_INCLUDE_DIRS})


add_executable(drive_and_map_main src/drive_and_map.cpp include/echo_handler.hpp include/json.hpp src/pose.h src/pose.cpp src/scan_match.h src/scan_match.cpp)
target_link_libraries(drive_and_map_main ${catkin_LIBRARIES})

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")