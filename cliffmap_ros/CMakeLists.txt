cmake_minimum_required(VERSION 2.8.3)
project(cliffmap_ros)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  tf
  message_generation
)

find_package(Boost REQUIRED)

find_package(Eigen3 REQUIRED)

add_message_files(
  FILES
  CLiFFMapLocationMsg.msg
  CLiFFMapDistributionMsg.msg
  CLiFFMapMsg.msg
)

add_service_files(
  FILES
  GetCLiFFMap.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cliffmap_ros
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs tf message_runtime
  DEPENDS Boost Eigen3
)

###########
## Build ##
###########

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(cliffmap_ros
  src/cliffmap.cpp
  src/cliffmap_conversions.cpp
)

add_dependencies(cliffmap_ros cliffmap_ros_generate_messages_cpp)
target_link_libraries(cliffmap_ros ${catkin_LIBRARIES})

add_executable(cliffmap_server src/cliffmap_server.cpp)
target_link_libraries(cliffmap_server cliffmap_ros ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS cliffmap_ros cliffmap_server
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)
