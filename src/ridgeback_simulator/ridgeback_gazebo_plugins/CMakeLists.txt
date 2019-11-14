cmake_minimum_required(VERSION 2.8.3)
project(ridgeback_gazebo_plugins)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs std_srvs geometry_msgs nav_msgs tf message_generation)
include_directories(include ${catkin_INCLUDE_DIRS})

## Find gazebo
find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

## Find Boost
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})


catkin_package(
    CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs tf message_runtime
    INCLUDE_DIRS include
    LIBRARIES
)

###########
## Build ##
###########

add_library(ridgeback_ros_force_based_move src/ridgeback_ros_force_based_move.cpp)
target_link_libraries(ridgeback_ros_force_based_move ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(TARGETS
  ridgeback_ros_force_based_move
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
