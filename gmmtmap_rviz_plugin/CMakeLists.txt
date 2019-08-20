cmake_minimum_required(VERSION 2.8.3)
project(gmmtmap_rviz_plugin)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
   gmmtmap_ros
   roscpp
   rviz
)

set(CMAKE_AUTOMOC ON)

## This plugin includes Qt widgets, so we must include Qt.
## We'll use the version that rviz used so they are compatible.
if (rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
else ()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
endif ()

add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
   LIBRARIES gmmtmap_rviz_plugin
   CATKIN_DEPENDS roscpp gmmtmap_ros rviz
)

###########
## Build ##
###########

include_directories(
   ${catkin_INCLUDE_DIRS}
)

set(SRC_FILES
   src/gmmtmap_display.cpp
   src/gmmtmap_visual.cpp
)

add_library(gmmtmap_rviz_plugin ${SRC_FILES})
target_link_libraries(gmmtmap_rviz_plugin ${QT_LIBRARIES} ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS gmmtmap_rviz_plugin
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES
   plugin_description.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
