/*
 * CLiFF-map ROS msg to cliffmap data structurex
 *
 */
#include <cliffmap_ros/cliffmap.hpp>

namespace cliffmap_ros {
CLiFFMap mapFromROSMsg(const CLiFFMapMsg &cliffmap_msg) {
  CLiFFMap cliffmap;

  cliffmap.x_min_ = cliffmap_msg.x_min;
  cliffmap.x_max_ = cliffmap_msg.x_max;
  cliffmap.y_min_ = cliffmap_msg.y_min;
  cliffmap.y_max_ = cliffmap_msg.y_max;

  cliffmap.radius_ = cliffmap_msg.radius;
  cliffmap.resolution_ = cliffmap_msg.resolution;

  cliffmap.rows_ = cliffmap_msg.rows;
  cliffmap.columns_ = cliffmap_msg.columns;

  for (const auto &location : cliffmap_msg.locations)
    cliffmap.locations_.push_back(locationFromROSMsg(location));
  return cliffmap;
}

CLiFFMapLocation
locationFromROSMsg(const CLiFFMapLocationMsg &cliffmap_location_msg) {
  CLiFFMapLocation loc;
  loc.id = cliffmap_location_msg.id;
  loc.position[0] = cliffmap_location_msg.position[0];
  loc.position[1] = cliffmap_location_msg.position[1];

  loc.p = cliffmap_location_msg.p;
  loc.q = cliffmap_location_msg.q;

  for (const auto distribution : cliffmap_location_msg.distributions) {
    loc.distributions.push_back(distributionFromROSMsg(distribution));
  }
  return loc;
}

CLiFFMapDistribution distributionFromROSMsg(
    const CLiFFMapDistributionMsg &cliffmap_distribution_msg) {
  CLiFFMapDistribution dist;
  dist.mixing_factor = cliffmap_distribution_msg.mixing_factor;

  if (cliffmap_distribution_msg.mean.size() == 2) {
    dist.mean[0] = cliffmap_distribution_msg.mean[0];
    dist.mean[1] = cliffmap_distribution_msg.mean[1];
  } else {
    std::cout << "Error: empty or bad mean in message.";
  }

  if (cliffmap_distribution_msg.covariance.size() == 4) {
    dist.covariance[0] = cliffmap_distribution_msg.covariance[0];
    dist.covariance[1] = cliffmap_distribution_msg.covariance[1];
    dist.covariance[2] = cliffmap_distribution_msg.covariance[2];
    dist.covariance[3] = cliffmap_distribution_msg.covariance[3];
  } else {
    std::cout << "Error: empty or bad covariance in message.";
  }
  return dist;
}

CLiFFMapMsg mapToROSMsg(const CLiFFMap &cliffmap) {
  CLiFFMapMsg cliffmap_msg;

  cliffmap_msg.x_min = cliffmap.x_min_;
  cliffmap_msg.x_max = cliffmap.x_max_;
  cliffmap_msg.y_min = cliffmap.y_min_;
  cliffmap_msg.y_max = cliffmap.y_max_;

  cliffmap_msg.radius = cliffmap.radius_;
  cliffmap_msg.resolution = cliffmap.resolution_;

  cliffmap_msg.rows = cliffmap.rows_;
  cliffmap_msg.columns = cliffmap.columns_;

  for (const auto &location : cliffmap.locations_)
    cliffmap_msg.locations.push_back(locationToROSMsg(location));
  return cliffmap_msg;
}

CLiFFMapLocationMsg
locationToROSMsg(const CLiFFMapLocation &cliffmap_location) {
  CLiFFMapLocationMsg loc_msg;
  loc_msg.id = cliffmap_location.id;

  loc_msg.position.push_back(cliffmap_location.position[0]);
  loc_msg.position.push_back(cliffmap_location.position[1]);

  loc_msg.p = cliffmap_location.p;
  loc_msg.q = cliffmap_location.q;

  for (const auto distribution : cliffmap_location.distributions) {
    loc_msg.distributions.push_back(distributionToROSMsg(distribution));
  }
  return loc_msg;
}

CLiFFMapDistributionMsg
distributionToROSMsg(const CLiFFMapDistribution &cliffmap_distribution) {
  CLiFFMapDistributionMsg dist_msg;
  dist_msg.mixing_factor = cliffmap_distribution.mixing_factor;

  dist_msg.mean.push_back(cliffmap_distribution.mean[0]);
  dist_msg.mean.push_back(cliffmap_distribution.mean[1]);

  dist_msg.covariance.push_back(cliffmap_distribution.covariance[0]);
  dist_msg.covariance.push_back(cliffmap_distribution.covariance[1]);
  dist_msg.covariance.push_back(cliffmap_distribution.covariance[2]);
  dist_msg.covariance.push_back(cliffmap_distribution.covariance[3]);

  return dist_msg;
}
} // namespace cliffmap_ros
