cmake_minimum_required(VERSION 2.8.3)
project(map)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/src)

add_executable(map_server src/server.cpp include/echo_handler.hpp include/json.hpp src/world_state.h src/world_state.cpp)
target_link_libraries(map_server ${catkin_LIBRARIES})
add_dependencies(map_server beginner_tutorials_generate_messages_cpp)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")