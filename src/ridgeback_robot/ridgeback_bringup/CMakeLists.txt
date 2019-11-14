cmake_minimum_required(VERSION 2.8.3)
project(ridgeback_bringup)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES scripts/can-udp-bridge.conf scripts/can-udp-bridge.sh
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/install
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch/accessories.launch)
endif()
