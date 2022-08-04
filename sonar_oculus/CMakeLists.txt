cmake_minimum_required(VERSION 2.8.3)
project(sonar_oculus)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  message_generation
  dynamic_reconfigure
  cv_bridge
)

find_package( OpenCV REQUIRED)


add_message_files(
   FILES
   OculusFire.msg
   OculusPing.msg
   OculusFireDeprecated.msg
   OculusPingDeprecated.msg
   OculusPingUncompressed.msg
)

generate_messages(
   DEPENDENCIES
   sensor_msgs
)

generate_dynamic_reconfigure_options(
   cfg/OculusParams.cfg
 )

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/sonar.cpp src/OculusClient.cpp)

add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg)

add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${roslib_LIBRARIES}
   ${OpenCV_LIBRARIES}

)
