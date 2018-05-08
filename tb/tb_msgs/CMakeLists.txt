cmake_minimum_required(VERSION 2.8.3)
project(tb_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
)

add_service_files(DIRECTORY srv
  FILES
  SetBatteryLevel.srv
)

generate_messages()

catkin_package(
  CATKIN_DEPENDS message_runtime
)