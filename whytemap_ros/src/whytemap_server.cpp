/*
 *   Copyright (c) Chittaranjan Swaminathan, Tomas Vintr, Tomas Krajnik
 *   This file is part of whytemap_ros.
 *
 *   whytemap_ros is free software: you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public License as
 *   published by the Free Software Foundation, either version 3 of the License,
 *   or (at your option) any later version.
 *
 *   whytemap_ros is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with whytemap_ros.  If not, see
 *   <https://www.gnu.org/licenses/>.
 */

#include <ros/ros.h>
#include <whytemap_ros/GetWHyTeMap.h>
#include <whytemap_ros/whytemap.hpp>

whytemap_ros::WHyTeMapMsg whytemap;

bool callback(whytemap_ros::GetWHyTeMap::Request &req,
              whytemap_ros::GetWHyTeMap::Response &res) {
  res.whytemap = whytemap;
  return true;
}

int main(int argn, char* args[]) {
  ros::init(argn, args, "whytemap_server");

  if (argn < 2) {
    std::cout << "Usage whytemap_server <file_name>" << std::endl;
    return -1;
  }

  ros::NodeHandle nh("~");
  ros::NodeHandle nh_;

  std::string whytemap_topic_name = "/whytemap";
  std::string whytemap_service_name = "/get_whytemap";
  std::string whytemap_frame_id = "/map";

  nh.getParam("topic_name", whytemap_topic_name);
  nh.getParam("service_name", whytemap_service_name);
  nh.getParam("frame_id", whytemap_frame_id);

  ros::Publisher whytemap_pub =
      nh_.advertise<whytemap_ros::WHyTeMapMsg>(whytemap_topic_name, 10, true);
  ROS_INFO("WHyTeMap will be published when there is a subscriber.");

  ros::ServiceServer service_ =
      nh_.advertiseService(whytemap_service_name, callback);

  whytemap_ros::WHyTeMap map;
  map.setFrameID(whytemap_frame_id);
  map.readFromXML(args[1]);
  std::cout << map;
  whytemap = map.toROSMsg();

  whytemap_pub.publish(whytemap);

  ros::spin();
  return 0;
}