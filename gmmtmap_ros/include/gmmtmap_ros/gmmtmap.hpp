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
#include <gmmtmap_ros/GMMTMapMsg.h>

#include <vector>
#include <array>

#include <Eigen/Core>

namespace gmmtmap_ros {

struct GMMTMapCluster {
  /// Mixing factor
  double mixing_factor;

  /// Cluster means
  std::vector<std::array<double, 2>> mean;

  inline GMMTMapCluster() {}
  inline GMMTMapCluster(double pi, std::vector<std::array<double, 2>> mean) {
    this->mixing_factor = pi;
    this->mean = mean;
  }
};

class GMMTMap {
 public:

  explicit GMMTMap(const gmmtmap_ros::GMMTMapMsg& msg);
  explicit GMMTMap(const std::string& fileName) { readFromXML(fileName); }

  void readFromXML(const std::string& fileName);
  gmmtmap_ros::GMMTMapMsg toROSMsg();

  inline int getM() { return M_; }
  inline int getK() { return K_; }
  inline double getStdDev() { return stddev_; }
  inline std::string getFrameID() { return frame_id_; }

  inline void setFrameID(const std::string& frame_id) { this->frame_id_ = frame_id; }
 protected:
  int M_;
  int K_;
  double stddev_;
  std::vector<GMMTMapCluster> clusters_;
  std::string frame_id_;
};

typedef std::shared_ptr<GMMTMap> GMMTMapPtr;
typedef std::shared_ptr<const GMMTMap> GMMTMapConstPtr;

}