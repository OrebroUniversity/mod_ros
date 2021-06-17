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
#include <tf/transform_listener.h>

#include <cliffmap_ros/CLiFFMapMsg.h>
#include <cliffmap_ros/GetCLiFFMap.h>

#include <cliffmap_ros/cliffmap.hpp>
#include <iostream>

cliffmap_ros::CLiFFMapMsg cliffmap;
tf::StampedTransform mod_to_laser2d;

bool callback(cliffmap_ros::GetCLiFFMap::Request &req,
              cliffmap_ros::GetCLiFFMap::Response &res) {
  res.cliffmap = cliffmap;
  return true;
}

int main(int argn, char *argv[]) {

  ros::init(argn, argv, "cliffmap_server");

  tf::TransformListener tf_listener;

  auto time_now = ros::Time::now();

  if (argn < 2) {
    std::cout << "Usage cliffmap_server <file_name>" << std::endl;
    return -1;
  }

  ros::NodeHandle nh("~");
  ros::NodeHandle nh_;

  std::string cliffmap_topic_name = "/cliffmap";
  std::string cliffmap_service_name = "/get_cliffmap";
  std::string cliffmap_frame_id = "/map_mod";
  std::string map_frame_id = "/map";

  double x_max = 0.0, y_max = 0.0, x_min = 0.0, y_min = 0.0;
  nh.getParam("topic_name", cliffmap_topic_name);
  nh.getParam("service_name", cliffmap_service_name);
  nh.getParam("frame_id", cliffmap_frame_id);
  nh.getParam("map_frame_id", map_frame_id);
  nh.getParam("y_max", y_max);
  nh.getParam("y_min", y_min);
  nh.getParam("x_max", x_max);
  nh.getParam("x_min", x_min);

  try {
    tf_listener.waitForTransform(map_frame_id, cliffmap_frame_id, time_now,
                                 ros::Duration(5));
    tf_listener.lookupTransform(map_frame_id, cliffmap_frame_id, ros::Time(0),
                                mod_to_laser2d);
  } catch (tf::TransformException &ex) {
    ROS_ERROR_STREAM(
        "Error getting transform from map_laser2d to map_mod: " << ex.what());
    mod_to_laser2d.setIdentity();
  }

  ROS_INFO_STREAM("Transform received. Origin: "
                  << mod_to_laser2d.getOrigin()
                  << ", Rotation: " << mod_to_laser2d.getRotation());

  ros::Publisher cliffmap_pub =
      nh_.advertise<cliffmap_ros::CLiFFMapMsg>(cliffmap_topic_name, 10, true);
  ROS_INFO("CLiFFMap will be published when there is a subscriber.");

  ros::ServiceServer service_ =
      nh_.advertiseService(cliffmap_service_name, callback);

  cliffmap_ros::CLiFFMap map;
  map.setFrameID(cliffmap_frame_id);
  map.readFromXML(argv[1]);
  map.organizeAsGrid();

  cliffmap = cliffmap_ros::mapToROSMsg(map.transformCLiFFMap(
      mod_to_laser2d, "map_laser2d", x_max, y_max, x_min, y_min));

  cliffmap_pub.publish(cliffmap);

  ros::spin();
  return 0;
}
