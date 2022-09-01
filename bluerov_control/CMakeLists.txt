cmake_minimum_required(VERSION 2.8)
project(bluerov_control)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(catkin REQUIRED COMPONENTS
  rospy
  bluerov_bridge
  sensor_msgs
  geometry_msgs
  nav_core
)

catkin_package()

