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
 *   along with cliffmap_rviz_plugin.  If not, see
 *   <https://www.gnu.org/licenses/>.
 */

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <cliffmap_ros/cliffmap.hpp>

#include <iostream>

namespace cliffmap_ros {

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
      if (vLocProperty.first == "p") try {
          location.p = vLocation.second.get<double>("p");
        } catch (std::exception &ex) {
          ROS_WARN_STREAM_THROTTLE(
              1.0, "There was an exception trying to get 'p' value from xml: "
                       << ex.what());
          location.p = 1.0;
        }
      else
        location.p = 1.0;

      if (vLocProperty.first == "q") try {
          location.q = vLocation.second.get<double>("q");
        } catch (std::exception &ex) {
          ROS_WARN_STREAM_THROTTLE(
              1.0, "There was an exception trying to get 'q' value from xml: "
                       << ex.what());
          location.q = 1.0;
        }
      else
        location.q = 1.0;

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
}

CLiFFMapLocation CLiFFMap::at(size_t row, size_t col) const {
  if (row > rows_ || col > columns_) {
    std::cout << "WARNING CLiFFMap::at called with out-of-bounds indices. "
                 "Returning empty CLiFFMapLocation.";
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

void CLiFFMap::organizeAsGrid() {
  std::vector<CLiFFMapLocation> organizedLocations;

  columns_ = round((x_max_ - x_min_) / resolution_) + 1;
  rows_ = round((y_max_ - y_min_) / resolution_) + 1;

  organizedLocations.resize(rows_ * columns_);

  if (organizedLocations.size() != locations_.size()) {
    ROS_INFO_STREAM(
        "[CLiFFMap] Error in number of locations. We thought it was "
        << organizedLocations.size() << ", but it was " << locations_.size()
        << ".");
    return;
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

}  // namespace cliffmap_ros

std::ostream &operator<<(std::ostream &out,
                         const cliffmap_ros::CLiFFMapDistribution &dist) {
  out << "Mixing Factor: " << dist.mixing_factor << "\t";
  out << "Mean: [" << dist.mean[0] << "," << dist.mean[1] << "]" << std::endl;
  return out;
}

std::ostream &operator<<(std::ostream &out,
                         const cliffmap_ros::CLiFFMapLocation &loc) {
  out << "Position: [" << loc.position[0] << ", " << loc.position[1] << "]\n";
  for (const auto &dist : loc.distributions) out << "Distribution: " << dist;
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
