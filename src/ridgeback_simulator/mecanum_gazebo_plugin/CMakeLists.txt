cmake_minimum_required(VERSION 2.8.3)
project(mecanum_gazebo_plugin)

find_package(catkin REQUIRED COMPONENTS rosconsole roslint)
find_package(gazebo)

catkin_package(
  #  LIBRARIES mecanum_gazebo_plugin
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

#add_library(${PROJECT_NAME} src/mecanum_plugin)
#target_link_libraries(${PROJECT_NAME}
#  ${catkin_LIBRARIES}
#  ${GAZEBO_LIBRARIES}
#)

# roslint_cpp()
# roslint_add_test()

#install(TARGETS ${PROJECT_NAME}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#)
