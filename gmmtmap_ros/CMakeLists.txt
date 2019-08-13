cmake_minimum_required(VERSION 2.8.3)
project(gmmtmap_ros)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
   geometry_msgs
   roscpp
   std_msgs
   tf
   message_generation)

find_package(Boost REQUIRED
   COMPONENTS
   system
   chrono)

find_package(Eigen3 REQUIRED)

add_message_files(
   FILES
   GMMTMapMsg.msg
   GMMTMapClusterMsg.msg)

add_service_files(
   FILES
   GetGMMTMap.srv)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   geometry_msgs
   std_msgs)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES gmmtmap_ros
   CATKIN_DEPENDS geometry_msgs roscpp std_msgs tf message_runtime
   DEPENDS Boost Eigen3)

###########
## Build ##
###########

include_directories(
   include
   ${EIGEN3_INCLUDE_DIRS}
   ${catkin_INCLUDE_DIRS}
   ${Boost_INCLUDE_DIRS})

## Declare a C++ library
add_library(gmmtmap_ros
   src/gmmtmap.cpp)

add_dependencies(gmmtmap_ros gmmtmap_ros_generate_messages_cpp)
target_link_libraries(gmmtmap_ros ${catkin_LIBRARIES})

add_executable(gmmtmap_server src/gmmtmap_server.cpp)
target_link_libraries(gmmtmap_server gmmtmap_ros ${catkin_LIBRARIES})

add_executable(rtree_test src/rtree_test.cpp)
target_link_libraries(rtree_test ${Boost_LIBRARIES})

#############
## Install ##
#############

install(TARGETS gmmtmap_ros gmmtmap_server
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
   PATTERN ".svn" EXCLUDE)