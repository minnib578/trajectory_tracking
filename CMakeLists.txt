cmake_minimum_required(VERSION 2.8.3)
project(trajectory_tracking)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES trajectory_tracking
 CATKIN_DEPENDS gazebo_ros geometry_msgs nav_msgs roscpp rospy std_msgs tf
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(circle src/circle.cpp)
target_link_libraries(circle ${catkin_LIBRARIES})
add_dependencies(circle trajectory_tracking_generate_messages_cpp)

add_executable(straghtline src/straghtline.cpp)
target_link_libraries(straghtline ${catkin_LIBRARIES})
add_dependencies(straghtline trajectory_tracking_generate_messages_cpp)