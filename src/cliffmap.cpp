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
      // this->step_ = value.second.get<int> ("step");
    }
  }
  for (const auto &vLocation : pTree.get_child("map.locations")) {
    CLiFFMapLocation location;
    location.id = vLocation.second.get<unsigned int>("id");
    // location.p = vLocation.second.get<unsigned int>("p");
    // location.q = vLocation.second.get<unsigned int>("q");

    for (const auto &vLocProperty : vLocation.second.get_child("")) {

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
    this->locations.push_back(location);
  }
  std::cout << "Read a cliffmap from XML" << std::endl;
}
} //  namespace

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

  for (const auto &loc : map.getLocations()) {
    out << "Location: " << loc;
  }
  return out;
}
