cmake_minimum_required(VERSION 2.8.3)
project(rm_simulator)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  tf
  tf2
  roborts_msgs
)

catkin_package(
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(acc2vel_node src/acc2vel.cpp)
target_link_libraries(acc2vel_node
  ${catkin_LIBRARIES}
)
add_dependencies(acc2vel_node
  ${catkin_EXPORTED_TARGETS})
