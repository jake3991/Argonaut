cmake_minimum_required(VERSION 2.8.3)
project(kvh_gyro)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  message_generation
)

add_message_files(
   FILES
   gyro.msg
)

generate_messages(
   DEPENDENCIES
   geometry_msgs
)

catkin_package()
