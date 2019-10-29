#pragma once

#include <string>
#include <vector>

namespace whytemap_ros {

struct WHyTeMapCluster {
  double weight{0.0};
  std::vector<double> centroid;
  std::vector<double> precision_matrix;

  WHyTeMapCluster(long degree);
};

class WHyTeMap {

public:
  WHyTeMap() : spatial_dim_(4) {}
  ~WHyTeMap() = default;

  /**
   * Read a WHyTe-map xml and load it into the data structres.
   * @param fileName The file name for WHyTeMap xml.
   */
  void readFromXML(const std::string &fileName);

  /**
   * Obtain the likelihood value for a particular query.
   * @param time The time at which likelihood is to be computed.
   * @param x The x coordinate
   * @param y The y coordinate
   * @param heading The heading
   * @param speed The speed.
   * @return Likelihood of a certain instance of position and velocity at a
   * certain time.
   */
  double getLikelihood(double time, double x, double y, double heading,
                       double speed);

private:
  /// No of clusters in the WHyTeMap
  long no_clusters_;

  /// The periodicity
  long no_periods_;

  /// What's the dimension of the random variable?
  /// This is the number of spatial dimensions + 2 * periodicity
  long degree_;

  /// Number of spatial dimensions. It's 4 now. 2D position and 2D velocity
  const long spatial_dim_;

  /// Chosen periodicity for hypertime projection
  std::vector<double> periods_;

  /// Clusters in the WHyTeMap
  std::vector<WHyTeMapCluster> clusters_;
};
}