
/**
 * Copyright 2017 Chittaranjan Srinivas Swaminathan
 * Licence: GNU GPL version 3
 */

#pragma once

#include <array>
#include <cmath>
#include <vector>
#include <visualization_msgs/MarkerArray.h>

namespace cliffmap_ros {

/**
 * A Single semi-wrapped Gaussian distribution.
 *
 */
struct CLiFFMapDistribution {
  double mixing_factor;
  std::array<double, 2> mean;
  std::array<double, 4> covariance;

  /**
   * \brief Returns the mixing factor for this distribution component.
   *
   * @return
   */
  inline double getMixingFactor() const { return mixing_factor; }

  /**
   * \brief Returns the mean for this distribution component.
   *
   * @return
   */
  inline std::array<double, 2> getMean() const { return mean; }

  /**
   * \brief Returns the covariance for this distribution component.
   *
   * @return
   */
  inline std::array<double, 4> getCovariance() const { return covariance; }

  /**
   * \brief Returns the mean heading for this distribution component.
   *
   * @return
   */
  inline double getMeanHeading() const { return mean[0]; }

  /**
   * \brief Returns the mean speed for this distribution component.
   *
   * @return
   */
  inline double getMeanSpeed() const { return mean[1]; }
};

/**
 * A location in the cliffmap. Might contain multiple distributions.
 *
 */
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

  // Used only if we want to interpret this cliffmap as a grid.
  double resolution_;

  // (rows_ - 1) * resolution_ = y_max_ - y_min_
  double rows_;
  // (columns_ - 1) * resolution_ = x_max_ - x_min_
  double columns_;
  bool organized_{false};

  std::vector<CLiFFMapLocation> locations_;

  inline double index2x(size_t col) const {
    return (((double)col * resolution_) + x_min_);
  }
  inline double index2y(size_t row) const {
    return (((double)row * resolution_) + y_min_);
  }
  inline size_t x2index(double x) const {
    return std::round((x - x_min_) / resolution_);
  }
  inline size_t y2index(double y) const {
    return std::round((y - y_min_) / resolution_);
  }

public:
  inline CLiFFMap() {}
  inline CLiFFMap(const std::string &fileName) { readFromXML(fileName); }

  /**
   * Calling this function makes the locations accessible as a grid.
   * Hence locations is modified such that it can be accessed as a row major
   * grid.
   * Use appropriate functions to query the grid.
   */
  void organizeAsGrid();

  /**
   * Get the CLiFFMapLocation at (row,col). Need to call organizeAsGrid() first.
   */

  visualization_msgs::MarkerArray toVisualizationMarkers() const;

  CLiFFMapLocation at(size_t row, size_t col) const;

  CLiFFMapLocation operator()(double x, double y) const;

  void readFromXML(const std::string &fileName);

  inline bool isOrganized() const { return organized_; }

  inline double getXMin() const { return x_min_; }
  inline double getYMin() const { return y_min_; }
  inline double getXMax() const { return x_max_; }
  inline double getYMax() const { return y_max_; }
  inline double getRadius() const { return radius_; }
  inline double getResolution() const { return resolution_; }
  inline const std::vector<CLiFFMapLocation> &getLocations() const {
    return locations_;
  }
};

typedef std::shared_ptr<CLiFFMap> CLiFFMapPtr;
typedef std::shared_ptr<const CLiFFMap> CLiFFMapConstPtr;
} // namespace cliffmap_ros

std::ostream &operator<<(std::ostream &, const cliffmap_ros::CLiFFMap &);
std::ostream &operator<<(std::ostream &,
                         const cliffmap_ros::CLiFFMapLocation &);
std::ostream &operator<<(std::ostream &,
                         const cliffmap_ros::CLiFFMapDistribution &);
