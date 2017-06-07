cmake_minimum_required(VERSION 2.8.3)
project(ws_test)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(server_base src/server.cpp include/echo_handler.hpp)
target_link_libraries(server_base ${catkin_LIBRARIES})
add_dependencies(server_base beginner_tutorials_generate_messages_cpp)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")