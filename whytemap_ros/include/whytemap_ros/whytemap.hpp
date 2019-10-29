#pragma once

#include <string>
#include <vector>

namespace whytemap_ros {

struct WHyTeMapCluster {
  double weight{0.0};
  std::vector<double> centroid;
  std::vector<double> precision_matrix;

  WHyTeMapCluster(long degree);

  /**
   * Get the precision matrix as an Eigen::MatrixXd.
   * @return The precision matrix as an Eigen Matrix.
   */
  inline Eigen::MatrixXd getPrecisionMatrix() const {
    std::vector<double> copy_precision_matrix = precision_matrix;
    // We can do this since we know centroid.size() = degree
    Eigen::MatrixXd copy_precision_matrix_eigen = Eigen::Map<Eigen::MatrixXd>(
        copy_precision_matrix.data(), centroid.size(), centroid.size());
    return copy_precision_matrix_eigen;
  }

  /**
   * Get the centroid as an Eigen::VectorXd.
   * @return The centroid as an Eigen Vector.
   */
  inline Eigen::VectorXd getCentroid() const {
    std::vector<double> copy_centroid = centroid;
    Eigen::VectorXd copy_centroid_eigen =
        Eigen::Map<Eigen::VectorXd>(copy_centroid.data(), centroid.size());
    return copy_centroid_eigen;
  }
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