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
#include <stdio.h>

#include <gsl/gsl_cdf.h>
#include <gsl/gsl_randist.h>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include <whytemap_ros/whytemap.hpp>

namespace whytemap_ros {

WHyTeMapCluster::WHyTeMapCluster(long degree) {
  this->centroid.resize(degree);
  this->precision_matrix.resize(degree * degree);
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
    WHyTeMapCluster cluster(no_periods_);
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

double WHyTeMap::getLikelihood(double time, double x, double y, double heading,
                               double speed) {
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

  std::vector<double>
      projection(degree_); // projection of tested vector
                                                // into warped hypertime space
  std::vector <double> shifted(degree_); // temporal variable, projection minus centre
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
    for (int i = 0; i < degree_; i++) {
      shifted[i] = projection[i] - this->clusters_[c].centroid[i];
    }
    distance = 0.0;
    for (int j = 0; j < degree_; j++) {
      tmp = 0.0;
      for (int i = 0; i < degree_; i++) {
        tmp = tmp + this->clusters_[c].precision_matrix[i * degree_ + j] * shifted[i];
      }
      tmp = tmp * shifted[j];
      distance += tmp;
    }
    /*probability of occurrence from the point of view of every cluster
     * (distribution estimation)*/
    prob += gsl_cdf_chisq_Q(distance, (double)degree_) * this->clusters_[c].weight;
  }
  /*return sum of particular and weighted probabilities*/
  return prob;
}

}