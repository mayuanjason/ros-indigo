cmake_minimum_required(VERSION 2.8.3)
project(tb_nav)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  dynamic_reconfigure
  geometry_msgs
  move_base_msgs
  nav_msgs
  roscpp
  rospy
  tf
  visualization_msgs
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/CalibrateLinear.cfg
  cfg/CalibrateAngular.cfg
)

catkin_package(
 CATKIN_DEPENDS actionlib dynamic_reconfigure geometry_msgs move_base_msgs nav_msgs roscpp rospy tf visualization_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)