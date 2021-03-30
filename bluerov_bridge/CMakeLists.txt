cmake_minimum_required(VERSION 2.8)
project(bluerov_bridge)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(catkin REQUIRED COMPONENTS
)

catkin_package(
)

catkin_python_setup()
