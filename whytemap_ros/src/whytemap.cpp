/*
 *   Copyright (c) Tomas Vintr, Tomas Krajnik
 *   This file is part of whytemap_ros.
 *
 *   whytemap_ros is free software: you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public License as
 *   published by the Free Software Foundation, either version 3 of the License,
 *   or (at your option) any later version.
 *
 *   whytemap_ros is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with whytemap_ros.  If not, see
 *   <https://www.gnu.org/licenses/>.
 */

#include <cmath>

#include <gsl/gsl_cdf.h>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include <whytemap_ros/whytemap.hpp>

namespace whytemap_ros {

WHyTeMapCluster::WHyTeMapCluster(long degree) {
  this->centroid.resize(degree);
  this->precision_matrix.resize(degree * degree);
}

WHyTeMapCluster::WHyTeMapCluster(const whytemap_ros::WHyTeMapClusterMsg &msg) {
  this->weight = msg.weight;
  this->centroid = msg.centroid;
  this->precision_matrix = msg.precision_matrix;
}

WHyTeMapClusterMsg WHyTeMapCluster::toROSMsg() const {
  WHyTeMapClusterMsg msg;
  msg.precision_matrix = this->precision_matrix;
  msg.centroid = this->centroid;
  msg.weight = this->weight;
  return msg;
}

void WHyTeMap::readFromXML(const std::string &fileName) {
  boost::property_tree::ptree pTree;

  boost::property_tree::read_xml(fileName, pTree);

  // reading data shapes
  no_clusters_ = pTree.get<long>("root.no_clusters");
  no_periods_ = pTree.get<long>("root.no_periods");

  // Spatial dimension is 4 + 2 dimensions for each period.
  degree_ = spatial_dim_ + no_periods_ * 2;

  // These values are redundant:
  // * c_shape_rows is the number of clusters (no_clusters_)
  // * c_shape_cols is the degree (degree_)
  // * prec_shape_0 is the number of clusters (no_clusters_)
  // * prec_shape_1 is the degree (degree_)
  // * prec_shape_2 is the degree (degree_)
  // long c_shape_rows = pTree.get<long>("root.C_shape.s_0");
  // long c_shape_cols = pTree.get<long>("root.C_shape.s_1");

  // Add empty clusters
  for (long i = 0; i < no_clusters_; i++) {
    WHyTeMapCluster cluster(degree_);
    this->clusters_.push_back(cluster);
  }

  // reading data values
  for (long rows = 0; rows < no_clusters_; rows++) {
    for (long cols = 0; cols < degree_; cols++) {
      clusters_[rows].centroid[cols] = pTree.get<double>(
          "root.C_values.v_" + std::to_string(rows * degree_ + cols));
    }
  }

  for (long i = 0; i < no_clusters_; i++) {
    clusters_[i].weight =
        pTree.get<double>("root.W_values.v_" + std::to_string(i));
  }

  for (long i = 0; i < no_clusters_; i++) // individual matrices
  {
    for (long j = 0; j < degree_; j++) // rows
    {
      for (long k = 0; k < degree_; k++) // cols
      {
        clusters_[i].precision_matrix[j * degree_ + k] =
            pTree.get<double>("root.PREC_values.v_" +
                              std::to_string((i * degree_ + j) * degree_ + k));
      }
    }
  }

  for (long i = 0; i < no_periods_; i++) {
    periods_.push_back(
        pTree.get<double>("root.periodicities_values.v_" + std::to_string(i)));
  }
}

WHyTeMapMsg WHyTeMap::toROSMsg() const {
  WHyTeMapMsg msg;
  msg.header.frame_id = this->frame_id_;
  msg.no_clusters = this->no_clusters_;
  msg.no_periods = this->no_periods_;
  msg.spatial_dim = this->spatial_dim_;
  msg.periods = this->periods_;
  for (const auto &cluster : this->clusters_) {
    msg.clusters.push_back(cluster.toROSMsg());
  }
  return msg;
}

std::vector<std::vector<double>> WHyTeMap::grid_weights = {
    {0.000000000000000000e+00, 2.479338842975206680e-02,
     4.628099173553719137e-01, 8.264462809917355601e-01,
     9.090909090909090606e-01, 8.264462809917355601e-01,
     4.628099173553719137e-01, 2.479338842975206680e-02,
     0.000000000000000000e+00},
    {2.479338842975206680e-02, 7.520661157024793875e-01,
     1.000000000000000000e+00, 1.000000000000000000e+00,
     1.000000000000000000e+00, 1.000000000000000000e+00,
     1.000000000000000000e+00, 7.520661157024793875e-01,
     2.479338842975206680e-02},
    {4.628099173553719137e-01, 1.000000000000000000e+00,
     1.000000000000000000e+00, 1.000000000000000000e+00,
     1.000000000000000000e+00, 1.000000000000000000e+00,
     1.000000000000000000e+00, 1.000000000000000000e+00,
     4.628099173553719137e-01},
    {8.264462809917355601e-01, 1.000000000000000000e+00,
     1.000000000000000000e+00, 1.000000000000000000e+00,
     1.000000000000000000e+00, 1.000000000000000000e+00,
     1.000000000000000000e+00, 1.000000000000000000e+00,
     8.264462809917355601e-01},
    {9.090909090909090606e-01, 1.000000000000000000e+00,
     1.000000000000000000e+00, 1.000000000000000000e+00,
     1.000000000000000000e+00, 1.000000000000000000e+00,
     1.000000000000000000e+00, 1.000000000000000000e+00,
     9.090909090909090606e-01},
    {8.264462809917355601e-01, 1.000000000000000000e+00,
     1.000000000000000000e+00, 1.000000000000000000e+00,
     1.000000000000000000e+00, 1.000000000000000000e+00,
     1.000000000000000000e+00, 1.000000000000000000e+00,
     8.264462809917355601e-01},
    {4.628099173553719137e-01, 1.000000000000000000e+00,
     1.000000000000000000e+00, 1.000000000000000000e+00,
     1.000000000000000000e+00, 1.000000000000000000e+00,
     1.000000000000000000e+00, 1.000000000000000000e+00,
     4.628099173553719137e-01},
    {2.479338842975206680e-02, 7.520661157024793875e-01,
     1.000000000000000000e+00, 1.000000000000000000e+00,
     1.000000000000000000e+00, 1.000000000000000000e+00,
     1.000000000000000000e+00, 7.520661157024793875e-01,
     2.479338842975206680e-02},
    {0.000000000000000000e+00, 2.479338842975206680e-02,
     4.628099173553719137e-01, 8.264462809917355601e-01,
     9.090909090909090606e-01, 8.264462809917355601e-01,
     4.628099173553719137e-01, 2.479338842975206680e-02,
     0.000000000000000000e+00}};

WHyTeMap::WHyTeMap(const WHyTeMapMsg &msg) {
  this->frame_id_ = msg.header.frame_id;
  this->no_clusters_ = msg.no_clusters;
  this->no_periods_ = msg.no_periods;
  this->spatial_dim_ = msg.spatial_dim;
  this->periods_ = msg.periods;
  this->degree_ = spatial_dim_ + no_periods_ * 2;
  this->clusters_.clear();
  for (const auto &cluster : msg.clusters) {
    this->clusters_.push_back(cluster);
  }
}

double WHyTeMap::getCost(double time, double x, double y, double heading,
                         double speed) const {
  /*
  calculates the probability of the occurrence of the tested vector using model
  input:
      tested vector (time, x, y, heading, speed)
      model:
          no_clusters .. number of clusters,
          no_periods .. number of chosen periodicities to build hypertime,
          periodicities .. set of the most influencing periodicities,
          C .. set of cluster centers
          W .. set of cluster weights
          PREC .. set of precision matices (inversed covariance matrices)
  */
  double distance;   // mahalanobis distance between each C and projection
  double prob = 0.0; // "probability" of the occurrence of tested position

  // std::vector<double> projection(degree_);
  // std::vector <double> shifted(degree_); //

  // Projection of tested vector into warped hypertime space
  Eigen::VectorXd projection(degree_);

  // Temporal variable, projection minus centre
  Eigen::VectorXd shifted = Eigen::VectorXd::Zero(degree_);

  /* filling the projection*/
  projection[0] = x;
  projection[1] = y;
  double a_x =
      cos(heading); // x axis of the new first basis vector parallel to heading
  double a_y =
      sin(heading); // y axis of the new first basis vector parallel to heading
  double b_x =
      -a_y; // x axis of the new second basis vector perpendicular to heading
  double b_y =
      a_x; // y axis of the new second basis vector perpendicular to heading


  /* projection of time into 'no_periods_' hypertime circles */
  for (int id_n_p = 0; id_n_p < no_periods_; id_n_p++) {
    projection[spatial_dim_ + 2 * id_n_p] =
        cos(time * 2.0 * M_PI / periods_[id_n_p]);
    projection[spatial_dim_ + 2 * id_n_p + 1] =
        sin(time * 2.0 * M_PI / periods_[id_n_p]);
  }

  /* calculation of probability of the human-robot encounter from different
     directions with different speeds in new basis; there is 9x9=27 velocities
     of humans weighted by 'grid_weights' in a way, that corresponds to the
     circle neighborhood of robot in grid; probabilities gathered at these "27
     points" represents estimation of integral over the probability function*/
  double ret = 0.0;
  for (int i = 0; i < 9; i++) {
    double cur_x = -2.0 + i * 0.5; // speed of humans in the direction of x
                                   // coordinate in the new basis
    for (int j = 0; j < 9; j++) {
      double cur_y = -2.0 + j * 0.5; // speed of humans in the direction of y
                                     // coordinate in the new basis
      projection[2] =
          a_x * cur_x - b_x * cur_y; // speed of humans in the direction of x
                                     // coordinate in the default basis
      projection[3] =
          a_y * cur_x + b_y * cur_y; // speed of humans in the direction of y
                                     // coordinate in the default basis

      /* calculate mahalanobis distance between projection and every cluster
       * centre*/
      prob = 0.0;
      for (int c = 0; c < no_clusters_; c++) {
        // Eigen for better readability of code.
        Eigen::MatrixXd precision = this->clusters_[c].getPrecisionMatrix();
        Eigen::VectorXd centroid = this->clusters_[c].getCentroid();

        // Vectorized
        shifted = projection - centroid;
        distance = shifted.transpose() * precision * shifted;

        /* probability of occurrence from the point of view of every cluster;
         * (distribution estimation);
           sum of particular and weighted probabilities */
        prob += gsl_cdf_chisq_Q(distance, (double)degree_) *
                this->clusters_[c].weight;
      }
      /*
      cost raises with the radius around the velocity of the robot;
      it is weighted by the probability of human velocity;
      grid_weights transform retangular grid into the "circle neighbourhood".
      */
      ret += sqrt(pow(cur_y, 2) + pow(speed - cur_x, 2)) * prob *
             grid_weights[i][j];
    }
  }

  /* return sum of costs, i.e., integral over the human velocity space */
  return ret;
}

double WHyTeMap::getLikelihood(double time, double x, double y, double heading,
                               double speed) const {
  /*
  calculates the probability of the occurrence of the tested vector using model
  input:
      tested vector (time, x, y, heading, speed)
      model:
          no_clusters .. number of clusters,
          no_periods .. number of chosen periodicities to build hypertime,
          periodicities .. set of the most influencing periodicities,
          C .. set of cluster centers
          W .. set of cluster weights
          PREC .. set of precision matices (inversed covariance matrices)
  */
  double distance;   // mahalanobis distance between each C and projection
  double tmp;        // subtotal during distance calculation
  double prob = 0.0; // "probability" of the occurrence of tested position

  // std::vector<double> projection(degree_);
  // std::vector <double> shifted(degree_); //

  // Projection of tested vector into warped hypertime space
  Eigen::VectorXd projection(degree_);

  // Temporal variable, projection minus centre
  Eigen::VectorXd shifted = Eigen::VectorXd::Zero(degree_);

  /* filling the projection*/
  projection[0] = x;
  projection[1] = y;
  projection[2] = cos(heading) * speed; // velocity in the direction of x
  projection[3] = sin(heading) * speed; // velocity in the direction of y

  for (int id_n_p = 0; id_n_p < no_periods_; id_n_p++) {
    projection[spatial_dim_ + 2 * id_n_p] =
        cos(time * 2.0 * M_PI / periods_[id_n_p]);
    projection[spatial_dim_ + 2 * id_n_p + 1] =
        sin(time * 2.0 * M_PI / periods_[id_n_p]);
  }

  /* calculate mahalanobis distance between projection and every cluster
   * centre*/
  for (int c = 0; c < no_clusters_; c++) {

    // Eigen for better readability of code.
    Eigen::MatrixXd precision = this->clusters_[c].getPrecisionMatrix();
    Eigen::VectorXd centroid = this->clusters_[c].getCentroid();

    // Vectorized
    shifted = projection - centroid;

    distance = shifted.transpose() * precision * shifted;

    /*probability of occurrence from the point of view of every cluster
     * (distribution estimation)*/
    prob +=
        gsl_cdf_chisq_Q(distance, (double)degree_) * this->clusters_[c].weight;
  }
  /*return sum of particular and weighted probabilities*/
  return prob;
}

WHyTeMapMsg WHyTeMapClient::get() {
  GetWHyTeMap msg;
  if (!whytemap_client.call(msg)) {
    ROS_ERROR_STREAM(
        "Failed to call WHyTe-Map msg. Service call failed. Empty map "
        "returned.");
    return WHyTeMapMsg();
  }
  ROS_INFO_STREAM("Got a WHyTe-Map msg.");
  return msg.response.whytemap;
}

WHyTeMapClient::WHyTeMapClient(const std::string &service_name) {
  whytemap_client = nh.serviceClient<GetWHyTeMap>(service_name);
  whytemap_client.waitForExistence();
  ROS_INFO_STREAM("Connected to WHyTe-Map server.");
}

} // namespace whytemap_ros

std::ostream &operator<<(std::ostream &out,
                         const whytemap_ros::WHyTeMapCluster &clus) {
  out << "\tWHyTeMapCluster: " << std::endl;
  out << "\t\tWeight: " << clus.weight << std::endl;
  out << "\t\tCentroid: " << std::endl << clus.getCentroid() << std::endl;
  out << "\t\tPrecision Matrix: " << std::endl
      << clus.getPrecisionMatrix() << std::endl;
}

std::ostream &operator<<(std::ostream &out, const whytemap_ros::WHyTeMap &map) {
  out << "WHyTeMap: " << std::endl;
  out << "Periods : " << std::endl;
  for (const auto period : map.getPeriods()) {
    out << period;
    out << ", ";
  }
  out << "\b\b" << std::endl;
  out << "Clusters: " << std::endl;
  for (const auto cluster : map.getClusters()) {
    out << cluster;
  }
  out << "Spatial Dimenstion: " << map.getSpatialDim() << std::endl;
}