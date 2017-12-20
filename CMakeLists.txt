cmake_minimum_required(VERSION 2.8.3)
project(cliffmap_ros)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  tf
)

find_package(Boost REQUIRED)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   std_msgs
# )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cliffmap_ros
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs tf
  DEPENDS Boost
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(cliffmap_ros
  src/cliffmap.cpp
)

add_executable(test_xml
  src/test_xml.cpp
)
target_link_libraries(test_xml cliffmap_ros)

