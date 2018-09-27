#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <cliffmap_ros/cliffmap.hpp>

#include <iostream>

namespace cliffmap_ros {

visualization_msgs::MarkerArray CLiFFMap::toVisualizationMarkers() const {

  visualization_msgs::MarkerArray ma;
  unsigned int id = 0;
  for (const auto &location : locations_) {
    const auto &x = location.position[0];
    const auto &y = location.position[1];
    for (const auto &distribution : location.distributions) {
      const auto &speed = distribution.getMeanSpeed();
      const auto &theta = distribution.getMeanHeading();

      visualization_msgs::Marker m;
      m.header.frame_id = "map";
      m.header.stamp = ros::Time::now();
      m.ns = "cliffmap";
      m.id = id++;
      m.type = visualization_msgs::Marker::ARROW;
      m.action = visualization_msgs::Marker::ADD;
      m.pose.position.x = x;
      m.pose.position.y = y;
      m.pose.position.z = 0.0;
      m.pose.orientation.x = 0.0;
      m.pose.orientation.y = 0.0;
      m.pose.orientation.w = cos(theta / 2.0);
      m.pose.orientation.z = sin(theta / 2.0);
      m.lifetime = ros::Duration(0);

      m.color.a = 1.0;
      m.color.b = 0.35;
      m.color.r = location.q;
      m.color.g = 0.35;

      m.scale.x = speed / 5.0;
      m.scale.y = 0.035;
      m.scale.z = 0.015;

      ma.markers.push_back(m);
    }
  }
  return ma;
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
    //location.p = vLocation.second.get<double>("p");
    //location.q = vLocation.second.get<double>("q");

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
    printf("ERROR ORGANIZING CLiFFMap: Error in number of locations. We "
           "thought it was %lu, but it was %lu.",
           organizedLocations.size(), locations_.size());
    return;
  }

  for (const CLiFFMapLocation &location : locations_) {
    size_t r = y2index(location.position[1]);
    size_t c = x2index(location.position[0]);

    // printf("\nR: %u, C: %u", r, c);
    // printf("\n(x,y) = (%lf,%lf)", location.position[0],
    // location.position[1]);

    organizedLocations[r * columns_ + c] = location;
  }
  locations_ = organizedLocations;
  organized_ = true;

  int qs = 0;
  for (const auto &loc : locations_) {
    if (loc.q < 0.5) {
      qs++;
    }
  }

  printf("\n%d locations are less motion locations.", qs);
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

  out << "XMin: " << map.getXMin() << "\n"
      << "XMax: " << map.getXMax() << "\n"
      << "YMin: " << map.getYMin() << "\n"
      << "YMax: " << map.getYMax() << "\n";

  for (const auto &loc : map.getLocations()) {
    out << "Location: " << loc;
  }
  return out;
}
