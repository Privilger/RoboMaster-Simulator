cmake_minimum_required(VERSION 2.8.3)
project(puma_motor_driver)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_updater
  puma_motor_msgs
  roscpp
  sensor_msgs
  serial
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES puma_motor_driver
  CATKIN_DEPENDS diagnostic_updater puma_motor_msgs roscpp sensor_msgs serial
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(puma_motor_driver
  src/driver.cpp
  src/serial_gateway.cpp
  src/socketcan_gateway.cpp
  src/multi_driver_node.cpp
  src/diagnostic_updater.cpp
)

target_link_libraries(puma_motor_driver
  ${catkin_LIBRARIES}
)
add_dependencies(puma_motor_driver puma_motor_msgs_gencpp)

add_executable(puma_multi_controller_node src/multi_controller_node.cpp)

target_link_libraries(puma_multi_controller_node
  puma_motor_driver
  ${catkin_LIBRARIES}
)

# Mark executables and/or libraries for installation
install(TARGETS puma_motor_driver puma_multi_controller_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  file(GLOB LINT_SRCS
    src/*.cpp
    include/puma_motor_driver/driver.h
    include/puma_motor_driver/gateway.h
    include/puma_motor_driver/message.h
    include/puma_motor_driver/serial_gateway.h
    include/puma_motor_driver/socketcan_gateway.h
    include/puma_motor_driver/multi_driver_node.h
    include/puma_motor_driver/diagnostic_updater.h
    test/*.cpp)
  roslint_cpp(${LINT_SRCS})
  roslint_add_test()

  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch/mecanum.launch)
  roslaunch_add_file_check(launch/omni.launch)

  catkin_add_gtest(fixed_point_test test/fixed_point_test.cpp)
endif()
