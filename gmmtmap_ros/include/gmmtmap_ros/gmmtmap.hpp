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

#pragma once

#include <ros/ros.h>
#include <ros/console.h>

#include <gmmtmap_ros/GMMTMapMsg.h>
#include <gmmtmap_ros/GetGMMTMap.h>

#include <array>
#include <vector>

#include <Eigen/Core>

#include <boost/chrono.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/index/rtree.hpp>

namespace gmmtmap_ros {

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

typedef bg::model::d2::point_xy<double> Point2D;
typedef bg::model::box<Point2D> Box;
typedef std::pair<Point2D, std::array<size_t, 2>> TreeValue;

struct GMMTMapCluster {
  /// Mixing factor
  double mixing_factor;

  /// Cluster means
  std::vector<std::array<double, 2>> mean;

  /// Approximate heading computed using an average.
  std::vector<double> heading;

  /// Default constructor.
  inline GMMTMapCluster() = default;

  /**
   * \brief Constructor of a cluster.
   * \param pi Mixing factor.
   * \param mean A vector of 2D means for the @K_ Gaussians.
   * \param heading A vector of approximate headings.
   */
  inline GMMTMapCluster(double pi,
                        const std::vector<std::array<double, 2>> &mean,
                        std::vector<double> heading) {
    this->mixing_factor = pi;
    this->mean = mean;
    this->heading = heading;
  }
};

class GMMTMap {
public:
  /**
   * \brief Constructor converts a ROS Message directly to a GMMTMap object.
   * \param msg The ROS message to convert.
   */
  GMMTMap(const gmmtmap_ros::GMMTMapMsg &msg);

  /**
   * \brief Constructor that reads a GMMTMap from an xml file.
   * \param fileName Filename of XML file.
   */
  explicit GMMTMap(const std::string &fileName) { readFromXML(fileName); }

  /**
   * \brief Read a GMMTMap from an xml file.
   * \param fileName Filename of XML file.
   */
  void readFromXML(const std::string &fileName);

  /**
   * \brief Convert this GMMTMap object to a ROS message.
   * \return The converted ROS message.
   */
  gmmtmap_ros::GMMTMapMsg toROSMsg() const;

  /**
   * \brief Prepares the Boost RTree and computes all headings as well.
   */
  void computeHeadingAndConstructRTree();

  /**
   * \brief Computes closest points (within a radius of @stddev_) from each
   * motion pattern.
   * \param x The x coordinate of the position for which the
   * query needs to be done.
   * \param y The y coordinate of the position for which
   * the query needs to be done.
   * \return Closest points with their cluster_id
   * and 'k'.
   */
  std::vector<TreeValue> getNearestNeighbors(double x, double y) const;

  inline std::vector<TreeValue> operator()(double x, double y) const {
    return this->getNearestNeighbors(x, y);
  };

  /**
   * \brief Get the number of motion patterns in this GMMT-map.
   * \return The number of motion patterns.
   */
  inline int getM() const { return M_; }

  /**
   * \brief Get the number of Gaussians in each motion pattern.
   * \return The number of Gaussians.
   */
  inline int getK() const { return K_; }

  /**
   * \brief Get the standard deviation of each Gaussian.
   * \return The standard deviation.
   */
  inline double getStdDev() const { return stddev_; }

  /**
   * \brief Get the Frame ID used in ROS messages.
   * \return ROS message header.frame_id.
   */
  inline std::string getFrameID() const { return frame_id_; }

  /**
   * \brief Set the frame ID in ROS message.
   */
  inline void setFrameID(const std::string &frame_id) {
    this->frame_id_ = frame_id;
  }

  /**
   * \brief Get the mixing factor for a certain cluster index.
   * @param cluster_id The index of the cluster.
   * @return The mixing factor for that cluster.
   */
  inline double getMixingFactorByClusterID(size_t cluster_idx) {
    if(cluster_idx >= this->clusters_.size()) {
      ROS_ERROR("getMixingFactorByClusterID() called with cluster_id >= number of clusters.");
      return 1.0;
    }

    return this->clusters_[cluster_idx].mixing_factor;
  }

  inline double getHeadingAtDist(size_t cluster_idx, size_t mean_idx) {
    if(cluster_idx >= this->clusters_.size()) {
      ROS_ERROR("getHeadingAtDist() called with cluster_idx >= number of clusters.");
      return 0.0;
    }

    if(mean_idx >= this->clusters_[cluster_idx].heading.size()) {
      ROS_ERROR("getHeadingAtDist() called with mean_idx >= number of traj-means in cluster.");
      return 0.0;
    }

    return this->clusters_[cluster_idx].heading[mean_idx];
  }

protected:
  /// The number of motion patterns in the GMM Trajectory Map.
  int M_;

  /// The number of Gaussians per motion pattern in the GMM Trajectory Map.
  int K_;

  /// The standard deviation used in the GMM Trajectory Map. The 2D Gaussians in
  /// each motion pattern are circular.
  double stddev_;

  /// A vector containing all the clusters (motion patterns).
  std::vector<GMMTMapCluster> clusters_;

  /// A tree used for
  bgi::rtree<TreeValue, bgi::quadratic<16>> rtree_;

  /// Frame ID in the ROS message. Might be used for transformation. Right now,
  /// it is only used to fill in the ROS message's header.frame_id.
  std::string frame_id_;
};

typedef std::shared_ptr<GMMTMap> GMMTMapPtr;
typedef std::shared_ptr<const GMMTMap> GMMTMapConstPtr;

class GMMTMapClient {
  ros::NodeHandle nh;
  ros::ServiceClient gmmtmap_client;

public:
  GMMTMapClient(const std::string& service_name = "get_gmmtmap");

  GMMTMapMsg get();
};

}