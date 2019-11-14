cmake_minimum_required(VERSION 2.8.3)
project(ridgeback_base)

find_package(catkin REQUIRED COMPONENTS
  controller_manager diagnostic_updater geometry_msgs hardware_interface puma_motor_driver puma_motor_msgs
  realtime_tools ridgeback_msgs roscpp rosserial_server sensor_msgs std_msgs)
find_package(Boost REQUIRED COMPONENTS thread chrono)

catkin_package()
catkin_add_env_hooks(50.ridgeback_find_mag_config
  SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(ridgeback_node
  src/ridgeback_base.cpp
  src/ridgeback_cooling.cpp
  src/ridgeback_diagnostic_updater.cpp
  src/ridgeback_hardware.cpp
  src/ridgeback_lighting.cpp
  )
target_link_libraries(ridgeback_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(ridgeback_node ${catkin_EXPORTED_TARGETS})

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS scripts/calibrate_compass scripts/compute_calibration
 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS ridgeback_node
 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch/base.launch)

  find_package(roslint REQUIRED)
  roslint_cpp()
  roslint_add_test()
endif()
