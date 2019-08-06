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


#include <gmmtmap_ros/gmmtmap.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

namespace gmmtmap_ros {

GMMTMap::GMMTMap(const gmmtmap_ros::GMMTMapMsg& msg) {
  this->K_ = msg.K;
  this->M_ = msg.M;
  this->stddev_ = msg.stddev;
  this->frame_id_ = msg.header.frame_id;

  for (const auto& cluster_msg : msg.clusters) {
    gmmtmap_ros::GMMTMapCluster cluster;
    cluster.mixing_factor = cluster_msg.mixing_factor;
    for(const auto& point : cluster_msg.mean)
      cluster.mean.push_back({point.x, point.y});
    this->clusters_.push_back(cluster);
  }
}

gmmtmap_ros::GMMTMapMsg GMMTMap::toROSMsg() {
  GMMTMapMsg msg;
  msg.K = this->K_;
  msg.M = this->M_;
  msg.stddev = this->stddev_;
  msg.header.frame_id = this->frame_id_;

  for(const auto& cluster : this->clusters_) {
    GMMTMapClusterMsg cluster_msg;
    cluster_msg.mixing_factor = cluster.mixing_factor;
    for(const auto& pt : cluster.mean) {
      geometry_msgs::Point p;
      p.x = pt[0];
      p.y = pt[1];
      cluster_msg.mean.push_back(p);
    }
    msg.clusters.push_back(cluster_msg);
  }
  return msg;
}

void GMMTMap::readFromXML(const std::string &fileName) {
  using boost::property_tree::ptree;
  ptree pTree;

  boost::property_tree::read_xml(fileName, pTree);

  this->K_ = pTree.get<int>("map.parameters.K");
  this->M_ = pTree.get<int>("map.parameters.M");
  this->stddev_ = pTree.get<double>("map.parameters.stddev");

  for(const auto& vCluster : pTree.get_child("map.clusters")) {
    GMMTMapCluster cluster;
    cluster.mixing_factor = vCluster.second.get<double>("pi");
    for(const auto& vPoint : vCluster.second.get_child("mean")) {
      cluster.mean.push_back({vPoint.second.get<double>("x"), vPoint.second.get<double>("y")});
    }
    this->clusters_.push_back(cluster);
  }

  ROS_INFO("Read a GMMT-map with %d clusters each containing %d gaussians", this->M_, this->K_);
}

}