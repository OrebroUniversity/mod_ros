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

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <gmmtmap_ros/gmmtmap.hpp>

namespace gmmtmap_ros {

GMMTMap::GMMTMap(const gmmtmap_ros::GMMTMapMsg &msg) {
  this->K_ = msg.K;
  this->M_ = msg.M;
  this->stddev_ = msg.stddev;
  this->frame_id_ = msg.header.frame_id;

  for (size_t cluster_id = 0; cluster_id < msg.clusters.size(); cluster_id++) {

    auto cluster_msg = msg.clusters[cluster_id];

    gmmtmap_ros::GMMTMapCluster cluster;
    cluster.mixing_factor = cluster_msg.mixing_factor;
    for (size_t i = 0; i < cluster_msg.mean.size(); i++) {

      const auto &point = cluster_msg.mean[i];
      double heading;

      if (i == 0) {
        heading = atan2(cluster_msg.mean[i + 1].y - cluster_msg.mean[i].y,
                        cluster_msg.mean[i + 1].x - cluster_msg.mean[i].x);
      } else if (i + 1 == msg.clusters.size()) {
        heading = atan2(cluster_msg.mean[i].y - cluster_msg.mean[i - 1].y,
                        cluster_msg.mean[i].x - cluster_msg.mean[i - 1].x);
      } else {
        heading = atan2(cluster_msg.mean[i + 1].y - cluster_msg.mean[i - 1].y,
                        cluster_msg.mean[i + 1].x - cluster_msg.mean[i - 1].x);
      }

      cluster.heading.push_back(heading);
      cluster.mean.push_back({point.x, point.y});
      this->rtree_.insert(std::make_pair(
          Point2D(point.x, point.y), std::array<size_t, 2>({cluster_id, i})));
    }
    this->clusters_.push_back(cluster);
  }
}

gmmtmap_ros::GMMTMapMsg GMMTMap::toROSMsg() const {
  GMMTMapMsg msg;
  msg.K = this->K_;
  msg.M = this->M_;
  msg.stddev = this->stddev_;
  msg.header.frame_id = this->frame_id_;

  for (const auto &cluster : this->clusters_) {
    GMMTMapClusterMsg cluster_msg;
    cluster_msg.mixing_factor = cluster.mixing_factor;
    for (const auto &pt : cluster.mean) {
      geometry_msgs::Point p;
      p.x = pt[0];
      p.y = pt[1];
      cluster_msg.mean.push_back(p);
    }
    msg.clusters.push_back(cluster_msg);
  }
  return msg;
}

std::vector<TreeValue> GMMTMap::getNearestNeighbors(double x, double y) const {
  std::vector<TreeValue> returned;
  Point2D query_pt(x, y);
  Box query_box(Point2D(query_pt.get<0>() - this->stddev_,
                        query_pt.get<1>() - this->stddev_),
                Point2D(query_pt.get<0>() + this->stddev_,
                        query_pt.get<1>() + this->stddev_));

  ROS_INFO("Query box is: (%lf, %lf), (%lf, %lf)",
           query_box.max_corner().get<0>(), query_box.max_corner().get<1>(),
           query_box.min_corner().get<0>(), query_box.min_corner().get<1>());

  //  rtree_.query(bgi::satisfies([&](TreeValue V) {
  //                 return bg::distance(V.first, query_pt) < 20.5;
  //               }),
  //               std::back_inserter(returned));

  rtree_.query(bgi::within(query_box) && bgi::satisfies([&](TreeValue V) {
                 return bg::distance(V.first, query_pt) < this->stddev_;
               }),
               std::back_inserter(returned));

  std::sort(returned.begin(), returned.end(), [&](TreeValue a, TreeValue b) {
    return bg::distance(a.first, query_pt) < bg::distance(b.first, query_pt);
  });

  std::vector<int> mps;
  for (auto value = returned.begin(); value != returned.end();) {
    if (std::find(mps.begin(), mps.end(), value->second[0]) == mps.end()) {
      mps.push_back(value->second[0]);
      ++value;
    }
    else {
      value = returned.erase(value);
    }
  }

  return returned;
}

void GMMTMap::computeHeadingAndConstructRTree() {
  for (size_t cluster_id = 0; cluster_id < this->clusters_.size();
       cluster_id++) {
    auto cluster = this->clusters_[cluster_id];

    for (size_t i = 0; i < cluster.mean.size(); i++) {

      const auto &point = cluster.mean[i];
      double heading;

      if (i == 0) {
        heading = atan2(cluster.mean[i + 1][1] - cluster.mean[i][1],
                        cluster.mean[i + 1][0] - cluster.mean[i][0]);
      } else if (i + 1 == this->clusters_.size()) {
        heading = atan2(cluster.mean[i][1] - cluster.mean[i - 1][1],
                        cluster.mean[i][0] - cluster.mean[i - 1][0]);
      } else {
        heading = atan2(cluster.mean[i + 1][1] - cluster.mean[i - 1][1],
                        cluster.mean[i + 1][0] - cluster.mean[i - 1][0]);
      }

      cluster.heading.push_back(heading);
      this->rtree_.insert(std::make_pair(
          Point2D(point[0], point[1]), std::array<size_t, 2>({cluster_id, i})));
    }
  }
}

void GMMTMap::readFromXML(const std::string &fileName) {
  using boost::property_tree::ptree;
  ptree pTree;

  boost::property_tree::read_xml(fileName, pTree);

  this->K_ = pTree.get<int>("map.parameters.K");
  this->M_ = pTree.get<int>("map.parameters.M");
  this->stddev_ = pTree.get<double>("map.parameters.stddev");

  for (const auto &vCluster : pTree.get_child("map.clusters")) {
    GMMTMapCluster cluster;
    cluster.mixing_factor = vCluster.second.get<double>("pi");
    for (const auto &vPoint : vCluster.second.get_child("mean")) {
      cluster.mean.push_back(
          {vPoint.second.get<double>("x"), vPoint.second.get<double>("y")});
    }
    this->clusters_.push_back(cluster);
  }

  this->computeHeadingAndConstructRTree();
  ROS_INFO("Read a GMMT-map with %d clusters each containing %d gaussians",
           this->M_, this->K_);
}

GMMTMapClient::GMMTMapClient(const std::string &service_name) {
  gmmtmap_client = nh.serviceClient<GetGMMTMap>(service_name);
  gmmtmap_client.waitForExistence();
  ROS_INFO_STREAM("Connected to GMMT-map server");
}

GMMTMapMsg GMMTMapClient::get() {
  GetGMMTMap msg;
  if (!gmmtmap_client.call(msg)) {
    ROS_ERROR_STREAM("Failed to call GMMT-map server. Service call failed. "
                     "Empty map returned.");
    return GMMTMapMsg();
  }
  ROS_INFO_STREAM("Got a GMMT-map from server.");
  return msg.response.gmmtmap;
}

}