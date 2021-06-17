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

#include <Eigen/Dense>
#include <cmath>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <cliffmap_ros/cliffmap.hpp>

#include <iostream>

namespace cliffmap_ros {

CLiFFMap::CLiFFMap(const CLiFFMapMsg &cliffmap_msg) {
  frame_id_ = cliffmap_msg.header.frame_id;
  x_min_ = cliffmap_msg.x_min;
  x_max_ = cliffmap_msg.x_max;
  y_min_ = cliffmap_msg.y_min;
  y_max_ = cliffmap_msg.y_max;

  radius_ = cliffmap_msg.radius;
  resolution_ = cliffmap_msg.resolution;

  rows_ = cliffmap_msg.rows;
  columns_ = cliffmap_msg.columns;

  for (const auto &location : cliffmap_msg.locations)
    locations_.push_back(locationFromROSMsg(location));
}

void CLiFFMap::readFromXML(const std::string &fileName) {
  using boost::property_tree::ptree;
  ptree pTree;

  boost::property_tree::read_xml(fileName, pTree);

  // Read top-most tag
  for (const auto &value : pTree.get_child("map")) {
    if (value.first == "parameters") {
      this->x_min_ = value.second.get<double>("x_min");
      this->y_min_ = value.second.get<double>("y_min");
      this->x_max_ = value.second.get<double>("x_max");
      this->y_max_ = value.second.get<double>("y_max");
      this->radius_ = value.second.get<double>("radious");
      this->resolution_ = value.second.get<double>("step");
    }
  }
  for (const auto &vLocation : pTree.get_child("map.locations")) {
    CLiFFMapLocation location;
    location.id = vLocation.second.get<size_t>("id");

    for (const auto &vLocProperty : vLocation.second.get_child("")) {
      if (vLocProperty.first == "p")
        try {
          location.p = vLocation.second.get<double>("p");
        } catch (std::exception &ex) {
          ROS_WARN_STREAM_THROTTLE(
              1.0, "There was an exception trying to get 'p' value from xml: "
                       << ex.what());
          location.p = 1.0;
        }

      if (vLocProperty.first == "q")
        try {
          location.q = vLocation.second.get<double>("q");
        } catch (std::exception &ex) {
          ROS_WARN_STREAM_THROTTLE(
              1.0, "There was an exception trying to get 'q' value from xml: "
                       << ex.what());
          location.q = 1.0;
        }

      if (vLocProperty.first == "pose") {
        location.position[0] = vLocProperty.second.get<double>("x");
        location.position[1] = vLocProperty.second.get<double>("y");
      }

      if (vLocProperty.first == "distribution") {
        CLiFFMapDistribution dist;
        dist.mixing_factor = vLocProperty.second.get<double>("P");
        for (const auto &vDistribution : vLocProperty.second.get_child("")) {
          if (vDistribution.first == "M") {
            dist.mean[0] = vDistribution.second.get<double>("th");
            dist.mean[1] = vDistribution.second.get<double>("r");
          }

          if (vDistribution.first == "Cov") {
            dist.covariance[0] = vDistribution.second.get<double>("e_11");
            dist.covariance[1] = vDistribution.second.get<double>("e_12");
            dist.covariance[2] = vDistribution.second.get<double>("e_21");
            dist.covariance[3] = vDistribution.second.get<double>("e_22");
          }
        }
        location.distributions.push_back(dist);
      }
    }
    this->locations_.push_back(location);
  }
  std::cout << "Read a cliffmap from XML" << std::endl;

  // Frame ID:
  std::cout << "Frame ID for cliffmap is: " << frame_id_;
}

CLiFFMapLocation CLiFFMap::at(size_t row, size_t col) const {
  if (row >= rows_ || col >= columns_) {
    return CLiFFMapLocation();
  }

  return locations_[row * columns_ + col];
}

CLiFFMapLocation CLiFFMap::atId(size_t id) const {
  return locations_[id - (size_t)1];
}

CLiFFMapLocation CLiFFMap::operator()(double x, double y) const {
  size_t row = y2index(y);
  size_t col = x2index(x);
  return this->at(row, col);
}

double CLiFFMap::getLikelihood(double x, double y, double heading,
                               double speed) const {
  CLiFFMapLocation loc = (*this)(x, y);
  Eigen::Vector2d V;
  V[0] = heading;
  V[1] = speed;

  double likelihood = 0.0;

  for (const auto &dist : loc.distributions) {
    Eigen::Matrix2d Sigma;
    std::array<double, 4> sigma_array = dist.getCovariance();
    Sigma(0, 0) = sigma_array[0];
    Sigma(0, 1) = sigma_array[1];
    Sigma(1, 0) = sigma_array[2];
    Sigma(1, 1) = sigma_array[3];

    Eigen::Vector2d myu;
    myu[0] = atan2(sin(dist.getMeanHeading()), cos(dist.getMeanHeading()));
    myu[1] = dist.getMeanSpeed();

    double mahalanobis_sq = (V - myu).transpose() * Sigma.inverse() * (V - myu);

    likelihood += (1 / (2 * M_PI)) * (1 / sqrt(Sigma.determinant())) *
                  exp(-0.5 * mahalanobis_sq) * dist.getMixingFactor();
  }
  return likelihood;
}

double CLiFFMap::getBestHeading(double x, double y) const {
  CLiFFMapLocation loc = (*this)(x, y);

  double best_likelihood = 0.0;
  double best_heading = 0.0;
  for (const auto &dist : loc.distributions) {
    Eigen::Matrix2d Sigma;
    std::array<double, 4> sigma_array = dist.getCovariance();
    Sigma(0, 0) = sigma_array[0];
    Sigma(0, 1) = sigma_array[1];
    Sigma(1, 0) = sigma_array[2];
    Sigma(1, 1) = sigma_array[3];

    Eigen::Vector2d myu;
    myu[0] = atan2(sin(dist.getMeanHeading()), cos(dist.getMeanHeading()));
    myu[1] = dist.getMeanSpeed();

    double likelihood = (1 / (2 * M_PI)) * (1 / sqrt(Sigma.determinant())) *
                        dist.getMixingFactor();
    if (likelihood > best_likelihood) {
      best_likelihood = likelihood;
      best_heading = myu[0];
    }
  }
  return best_heading;
}

void CLiFFMap::organizeAsGrid() {
  std::vector<CLiFFMapLocation> organizedLocations;

  columns_ = round((x_max_ - x_min_) / resolution_) + 1;
  rows_ = round((y_max_ - y_min_) / resolution_) + 1;

  organizedLocations.resize(rows_ * columns_);

  if (organizedLocations.size() != locations_.size()) {
    ROS_WARN_STREAM(
        "[CLiFFMap] Error in number of locations. We thought it was "
        << organizedLocations.size() << ", but it was " << locations_.size()
        << ".");
    if (organizedLocations.size() < locations_.size())
      return;
    ROS_INFO_STREAM("Less is more. We have the space. Let's continue... ");
  }

  for (const CLiFFMapLocation &location : locations_) {
    size_t r = y2index(location.position[1]);
    size_t c = x2index(location.position[0]);

    organizedLocations[r * columns_ + c] = location;
  }
  locations_ = organizedLocations;
  organized_ = true;

  ROS_INFO_STREAM("[CLiFFMap] Organized a cliffmap with resolution: "
                  << getResolution() << " m/cell.");
}

CLiFFMapMsg CLiFFMapClient::get() {
  GetCLiFFMap msg;
  if (!cliffmap_client.call(msg)) {
    ROS_ERROR_STREAM(
        "Failed to call CLiFF-Map msg. Service call failed. Empty map "
        "returned.");
    return CLiFFMapMsg();
  }
  ROS_INFO_STREAM("Got a CLiFF-Map msg.");
  return msg.response.cliffmap;
}

CLiFFMap CLiFFMap::transformCLiFFMap(tf::StampedTransform &transform,
                                     const std::string &frame_id) {
  CLiFFMap transformedMap;
  transformedMap.setFrameID(frame_id);

  auto Rotation = transform.getBasis();
  auto Origin = transform.getOrigin();

  tf::Vector3 MaxValues, MaxValuesTransformed;
  tf::Vector3 MinValues, MinValuesTransformed;

  MaxValues.setX(this->x_max_);
  MaxValues.setY(this->y_max_);
  MaxValues.setZ(0.0);

  MinValues.setX(this->x_min_);
  MinValues.setY(this->y_min_);
  MinValues.setZ(0.0);

  MaxValuesTransformed = transform * MaxValues;
  MinValuesTransformed = transform * MinValues;

  transformedMap.x_max_ = MaxValuesTransformed.getX();
  transformedMap.y_max_ = MaxValuesTransformed.getY();
  transformedMap.x_min_ = MinValuesTransformed.getX();
  transformedMap.y_min_ = MinValuesTransformed.getY();

  for (const CLiFFMapLocation& l : this->locations_) {

    tf::Vector3 Position;
    Position.setX(l.position[0]);
    Position.setY(l.position[1]);
    Position.setZ(0.0);

    auto new_position = transform * Position;
    double new_theta, bleh, blah;

    Rotation.getRPY(bleh, blah, new_theta);

    CLiFFMapLocation l_new;

    l_new.position[0] = new_position.getX();
    l_new.position[1] = new_position.getY();

    for (const auto &dist : l.distributions) {

      CLiFFMapDistribution dist_new;
      // Only the theta needs be transformed.
      dist_new.mean[0] = dist.mean[0] + new_theta;
      dist_new.mean[1] = dist.mean[1];
      dist_new.mixing_factor = dist.mixing_factor;

      l_new.distributions.push_back(dist_new);
    }

    transformedMap.locations_.push_back(l_new);
  }

  transformedMap.organizeAsGrid();
  return transformedMap;
}

CLiFFMapClient::CLiFFMapClient(const std::string &service_name) {
  cliffmap_client = nh.serviceClient<GetCLiFFMap>(service_name);
  cliffmap_client.waitForExistence();
  ROS_INFO_STREAM("Connected to CLiFF-Map server.");
}

} // namespace cliffmap_ros

std::ostream &operator<<(std::ostream &out,
                         const cliffmap_ros::CLiFFMapDistribution &dist) {
  out << "Mixing Factor: " << dist.mixing_factor << "\t";
  out << "Mean: [" << dist.mean[0] << "," << dist.mean[1] << "]" << std::endl;
  return out;
}

std::ostream &operator<<(std::ostream &out,
                         const cliffmap_ros::CLiFFMapLocation &loc) {
  out << "Position: [" << loc.position[0] << ", " << loc.position[1] << "]\n";
  for (const auto &dist : loc.distributions)
    out << "Distribution: " << dist;
  return out;
}

std::ostream &operator<<(std::ostream &out, const cliffmap_ros::CLiFFMap &map) {
  out << "XMin: " << map.getXMin() << "\n"
      << "XMax: " << map.getXMax() << "\n"
      << "YMin: " << map.getYMin() << "\n"
      << "YMax: " << map.getYMax() << "\n";

  for (const auto &loc : map.getLocations()) {
    out << "Location: " << loc;
  }
  return out;
}
