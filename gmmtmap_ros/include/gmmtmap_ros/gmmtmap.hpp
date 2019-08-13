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

#include <gmmtmap_ros/GMMTMapMsg.h>
#include <ros/ros.h>

#include <array>
#include <vector>

#include <Eigen/Core>

#include <boost/chrono.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/index/rtree.hpp>

namespace gmmtmap_ros {

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

typedef bg::model::d2::point_xy<double> Point2D;
typedef bg::model::box<Point2D> Box;

struct GMMTMapCluster {
  /// Mixing factor
  double mixing_factor;

  /// Cluster means
  std::vector<std::array<double, 2>> mean;

  std::vector<double> heading;

  inline GMMTMapCluster() = default;
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
  explicit GMMTMap(const gmmtmap_ros::GMMTMapMsg &msg);
  explicit GMMTMap(const std::string &fileName) { readFromXML(fileName); }

  void readFromXML(const std::string &fileName);
  gmmtmap_ros::GMMTMapMsg toROSMsg();

  void computeHeadingAndConstructRTree();

  std::vector<std::pair<Point2D, double>> getNearestNeighbors(double x,
                                                              double y);

  inline int getM() { return M_; }
  inline int getK() { return K_; }
  inline double getStdDev() { return stddev_; }
  inline std::string getFrameID() { return frame_id_; }

  inline void setFrameID(const std::string &frame_id) {
    this->frame_id_ = frame_id;
  }

protected:
  int M_;
  int K_;
  double stddev_;
  std::vector<GMMTMapCluster> clusters_;
  bgi::rtree<std::pair<Point2D, double>, bgi::quadratic<16>> rtree_;
  std::string frame_id_;
};

typedef std::shared_ptr<GMMTMap> GMMTMapPtr;
typedef std::shared_ptr<const GMMTMap> GMMTMapConstPtr;

}