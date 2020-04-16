cmake_minimum_required(VERSION 2.8.3)
project(puma_motor_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_message_files(
  FILES
  MultiStatus.msg
  Status.msg
  MultiFeedback.msg
  Feedback.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS std_msgs message_runtime)
