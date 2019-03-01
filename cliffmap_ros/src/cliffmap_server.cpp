#include <ros/ros.h>

#include <cliffmap_ros/CLiFFMapMsg.h>
#include <cliffmap_ros/GetCLiFFMap.h>

#include <cliffmap_ros/cliffmap.hpp>
#include <iostream>

cliffmap_ros::CLiFFMapMsg cliffmap;

bool callback(cliffmap_ros::GetCLiFFMap::Request &req,
              cliffmap_ros::GetCLiFFMap::Response &res) {
  res.cliffmap = cliffmap;
  return true;
}

int main(int argn, char *argv[]) {

  ros::init(argn, argv, "cliffmap_server");

  if (argn < 2) {
    std::cout << "Usage cliffmap_server <file_name>" << std::endl;
    return -1;
  }

  ros::NodeHandle nh;
  ros::Publisher cliffmap_pub =
      nh.advertise<cliffmap_ros::CLiFFMapMsg>("cliffmap", 10, true);
  ROS_INFO("CLiFFMap will be published when there is a subscriber.");

  ros::ServiceServer service_ = nh.advertiseService("get_cliffmap", callback);

  cliffmap_ros::CLiFFMap map(argv[1]);
  map.organizeAsGrid();
  cliffmap = mapToROSMsg(map);

  cliffmap_pub.publish(cliffmap);

  ros::spin();
  return 0;
}
