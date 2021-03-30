cmake_minimum_required(VERSION 2.8.3)
project(bar30_depth)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  bluerov_bridge
  message_generation
)

add_message_files(
   FILES
   Depth.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package()
