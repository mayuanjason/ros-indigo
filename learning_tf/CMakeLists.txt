cmake_minimum_required(VERSION 2.8.3)
project(learning_tf)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  turtlesim
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(turtle_tf_broadcaster src/turtle_tf_broadcaster.cpp)
add_executable(turtle_tf_listener src/turtle_tf_listener.cpp)
add_executable(frame_tf_broadcaster src/frame_tf_broadcaster.cpp)

target_link_libraries(turtle_tf_broadcaster ${catkin_LIBRARIES})
target_link_libraries(turtle_tf_listener ${catkin_LIBRARIES})
target_link_libraries(frame_tf_broadcaster ${catkin_LIBRARIES})

# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



