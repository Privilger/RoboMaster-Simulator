cmake_minimum_required(VERSION 2.8.3)
project(ridgeback_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

add_message_files(
  FILES
  Fans.msg
  Lights.msg
  RGB.msg
  Status.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS std_msgs message_runtime)
