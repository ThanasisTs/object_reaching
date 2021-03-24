cmake_minimum_required(VERSION 3.0.2)
project(object_reaching)

find_package(catkin REQUIRED COMPONENTS
  cartesian_state_msgs
  geometry_msgs
  roscpp
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES object_reaching
#  CATKIN_DEPENDS cartesian_state_msgs geometry_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable (main src/main.cpp)
target_link_libraries(main ${catkin_LIBRARIES})