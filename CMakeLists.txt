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

add_executable (robot_response src/robot_response.cpp)
target_link_libraries(robot_response ${catkin_LIBRARIES})

add_executable (robot_motion src/robot_motion.cpp)
target_link_libraries(robot_motion ${catkin_LIBRARIES})
