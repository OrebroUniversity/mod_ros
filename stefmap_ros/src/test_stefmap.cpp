#include <ros/ros.h>
#include <stefmap_ros/stefmap.hpp>

int main(int argn, char* args[]) {
  ros::init(argn, args, "test_stefmap");
  ROS_INFO_STREAM("Testing STeFMap client and conversions.");

  stefmap_ros::STeFMapClient c;
  stefmap_ros::STeFMap stefmap(c.get(1352265000.0, 2, -45, 55, -35, 30, 1));

  if (stefmap.isOrganized())
    ROS_INFO_STREAM("STeFMap is organized. Go home now.");
  else
    ROS_ERROR_STREAM("STeFMap isn't organized. Got work to do...");
  return 0;
}
