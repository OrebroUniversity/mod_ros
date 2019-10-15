/*
 *   Copyright (c) Chittaranjan Srinivas Swaminathan
 *   This file is part of cliffmap_ros.
 *
 *   cliffmap_ros is free software: you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public License as
 *   published by the Free Software Foundation, either version 3 of the License,
 *   or (at your option) any later version.
 *
 *   cliffmap_ros is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with cliffmap_ros.  If not, see
 *   <https://www.gnu.org/licenses/>.
 */

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

  ros::NodeHandle nh("~");
  ros::NodeHandle nh_;

  std::string cliffmap_topic_name = "/cliffmap";
  std::string cliffmap_service_name = "/get_cliffmap";
  std::string cliffmap_frame_id = "/map";

  nh.getParam("topic_name", cliffmap_topic_name);
  nh.getParam("service_name", cliffmap_service_name);
  nh.getParam("frame_id", cliffmap_frame_id);

  ros::Publisher cliffmap_pub =
      nh_.advertise<cliffmap_ros::CLiFFMapMsg>(cliffmap_topic_name, 10, true);
  ROS_INFO("CLiFFMap will be published when there is a subscriber.");

  ros::ServiceServer service_ = nh_.advertiseService(cliffmap_service_name, callback);

  cliffmap_ros::CLiFFMap map;
  map.setFrameID(cliffmap_frame_id);
  map.readFromXML(argv[1]);
  map.organizeAsGrid();
  cliffmap = mapToROSMsg(map);

  cliffmap_pub.publish(cliffmap);

  ros::spin();
  return 0;
}
