cmake_minimum_required(VERSION 2.8.3)
project(actionlib_tutorials)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  roscpp
  rospy
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES Fibonacci.action Averaging.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES actionlib_tutorials
#  CATKIN_DEPENDS actionlib actionlib_msgs message_generation roscpp rospy std_msgs
#  DEPENDS system_lib
  CATKIN_DEPENDS actionlib_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(fibonacci_server src/fibonacci_server.cpp)
add_executable(fibonacci_client src/fibonacci_client.cpp)
add_executable(averaging_server src/averaging_server.cpp)
add_executable(averaging_client src/averaging_client.cpp)

target_link_libraries(fibonacci_server ${catkin_LIBRARIES})
target_link_libraries(fibonacci_client ${catkin_LIBRARIES})
target_link_libraries(averaging_server ${catkin_LIBRARIES})
target_link_libraries(averaging_client ${catkin_LIBRARIES})

add_dependencies(fibonacci_server ${actionlib_tutorials_EXPORTED_TARGETS})
add_dependencies(fibonacci_client ${actionlib_tutorials_EXPORTED_TARGETS})
add_dependencies(averaging_server ${actionlib_tutorials_EXPORTED_TARGETS})
add_dependencies(averaging_client ${actionlib_tutorials_EXPORTED_TARGETS})