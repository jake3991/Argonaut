cmake_minimum_required(VERSION 2.8.3)
project(rti_dvl)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  message_generation
)

add_message_files(
   FILES
   DVL.msg
   Command.msg
   BottomTrack.msg
)

generate_messages(
   DEPENDENCIES
   geometry_msgs
)

catkin_package()
