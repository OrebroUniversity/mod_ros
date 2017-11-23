
/**
 * Copyright 2017 Chittaranjan Srinivas Swaminathan
 */

#include <array>
#include <vector>

namespace cliffmap_ros {

struct CLiFFMapDistribution {
  double mixing_factor;
  std::array<double, 2> mean;
  std::array<double, 4> covariance;

  inline double getMixingFactor() { return mixing_factor; }
  inline std::array<double, 2> getMean() { return mean; }
  inline std::array<double, 4> getCovariance() { return covariance; }
  inline double getMeanHeading() { return mean[0]; }
  inline double getMeanSpeed() { return mean[1]; }
};

struct CLiFFMapLocation {
  unsigned int id;
  std::array<double, 2> position;
  double p;
  double q;
  std::vector<CLiFFMapDistribution> distributions;
};

class CLiFFMap {
protected:
  double x_min_;
  double x_max_;
  double y_min_;
  double y_max_;

  double radius_;
  std::vector<CLiFFMapLocation> locations;

  void readFromXML(const std::string &fileName);

public:
  inline CLiFFMap(const std::string &fileName) { readFromXML(fileName); }

  inline double getXMin() const { return x_min_; }
  inline double getYMin() const { return y_min_; }
  inline double getXMax() const { return x_max_; }
  inline double getYMax() const { return y_max_; }
  inline double getRadius() const { return radius_; }
  inline const std::vector<CLiFFMapLocation> &getLocations() const {
    return locations;
  }
};
} // namespace cliffmap_ros

std::ostream &operator<<(std::ostream &, const cliffmap_ros::CLiFFMap &);
std::ostream &operator<<(std::ostream &,
                         const cliffmap_ros::CLiFFMapLocation &);
std::ostream &operator<<(std::ostream &,
                         const cliffmap_ros::CLiFFMapDistribution &);
