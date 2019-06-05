cmake_minimum_required(VERSION 2.8.3)
project(pedsim_scenarios)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY scenes/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scenes
  PATTERN ".svn" EXCLUDE)
