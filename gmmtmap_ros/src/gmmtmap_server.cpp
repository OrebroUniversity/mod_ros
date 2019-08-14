/*
 *   Copyright (c) Chittaranjan Srinivas Swaminathan
 *   This file is part of gmmtmap_ros.
 *
 *   gmmtmap_ros is free software: you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public License as
 *   published by the Free Software Foundation, either version 3 of the License,
 *   or (at your option) any later version.
 *
 *   gmmtmap_ros is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *   GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with gmmtmap_ros.  If not, see
 *   <https://www.gnu.org/licenses/>.
 */

#include <ros/ros.h>

#include <gmmtmap_ros/GetGMMTMap.h>
#include <gmmtmap_ros/gmmtmap.hpp>

gmmtmap_ros::GMMTMapMsg gmmtmap;

bool callback(gmmtmap_ros::GetGMMTMap::Request &req,
              gmmtmap_ros::GetGMMTMap::Response &res) {
  res.gmmtmap = ::gmmtmap;
  return true;
}

int main(int argn, char *args[]) {
  ros::init(argn, args, "gmmtmap_server");

  ros::NodeHandle nh_private("~");
  ros::NodeHandle nh;

  std::string gmmtmap_topic_name = "/gmmtmap";
  std::string gmmtmap_service_name = "/get_gmmtmap";
  std::string gmmtmap_frame_id = "map";

  nh_private.getParam("topic_name", gmmtmap_topic_name);
  nh_private.getParam("service_name", gmmtmap_service_name);
  nh_private.getParam("frame_id", gmmtmap_frame_id);

  ros::Publisher gmmtmap_pub =
      nh.advertise<gmmtmap_ros::GMMTMapMsg>(gmmtmap_topic_name, 10, true);
  ROS_INFO("gmmtmap will be published when there is a subscriber.");

  ros::ServiceServer service_ =
      nh.advertiseService(gmmtmap_service_name, callback);

  gmmtmap_ros::GMMTMap map(args[1]);
  map.setFrameID(gmmtmap_frame_id);
  gmmtmap = map.toROSMsg();
  gmmtmap_pub.publish(gmmtmap);

  ros::spin();
  return 0;
}
