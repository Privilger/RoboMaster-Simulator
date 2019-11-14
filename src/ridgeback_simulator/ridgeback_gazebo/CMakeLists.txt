cmake_minimum_required(VERSION 2.8.3)
project(ridgeback_gazebo)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package()

roslaunch_add_file_check(launch/ridgeback_world.launch)

install(DIRECTORY launch Media worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
