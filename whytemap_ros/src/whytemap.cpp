#include <cmath>
#include <stdio.h>

#include <gsl/gsl_cdf.h>
#include <gsl/gsl_randist.h>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include <whytemap_ros/whytemap.hpp>

WHyTeMap::WHyTeMap() {}

WHyTeMap::~WHyTeMap() {}

void WHyTeMap::readFromXML(const std::string &fileName) {
  boost::property_tree::ptree pTree;

  boost::property_tree::read_xml(fileName, pTree);

  // reading data shapes
  no_clusters = pTree.get<long>("root.no_clusters");
  no_periods = pTree.get<long>("root.no_periods");

  long c_shape_rows = pTree.get<long>("root.C_shape.s_0");
  long c_shape_cols = pTree.get<long>("root.C_shape.s_1");
  C.resize(c_shape_rows);
  for (long i = 0; i < c_shape_rows; i++)
    C[i].resize(c_shape_cols);

  long w_shape_size = pTree.get<long>("root.W_shape.s_0");
  W.resize(w_shape_size);

  long prec_shape_0 = pTree.get<long>("root.PREC_shape.s_0");
  long prec_shape_1 = pTree.get<long>("root.PREC_shape.s_1");
  long prec_shape_2 = pTree.get<long>("root.PREC_shape.s_2");
  PREC.resize(prec_shape_0); // resize to fit individual matrices
  for (long i = 0; i < prec_shape_0; i++) {
    PREC[i].resize(prec_shape_1); // resize to fit rows
    for (long j = 0; j < prec_shape_1; j++) {
      PREC[i][j].resize(prec_shape_2); // resize to fit cols
    }
  }

  long periodicities_size = pTree.get<long>("root.periodicities_shape.s_0");
  periodicities.resize(periodicities_size);

  // reading data values
  for (long rows = 0; rows < c_shape_rows; rows++) {
    for (long cols = 0; cols < c_shape_cols; cols++) {
      C[rows][cols] = pTree.get<double>(
          "root.C_values.v_" + std::to_string(rows * c_shape_cols + cols));
    }
  }

  for (long i = 0; i < w_shape_size; i++) {
    W[i] = pTree.get<double>("root.W_values.v_" + std::to_string(i));
  }

  for (long i = 0; i < prec_shape_0; i++) // individual matrices
  {
    for (long j = 0; j < prec_shape_1; j++) // rows
    {
      for (long k = 0; k < prec_shape_2; k++) // cols
      {
        PREC[i][j][k] = pTree.get<double>(
            "root.PREC_values.v_" +
            std::to_string((i * prec_shape_1 + j) * prec_shape_2 + k));
      }
    }
  }

  for (long i = 0; i < periodicities_size; i++) {
    periodicities[i] =
        pTree.get<double>("root.periodicities_values.v_" + std::to_string(i));
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
  long spatial_dim = 4;                          // pos_x, pos_y, vel_x, vel_y
  double degrees = spatial_dim + 2 * no_periods; // degrees of freedom for chi2
  double distance;   // mahalanobis distance between each C and projection
  double tmp;        // subtotal during distance calculation
  double prob = 0.0; // "probability" of the occurrence of tested position

  double
      projection[spatial_dim + 2 * no_periods]; // projection of tested vector
                                                // into warped hypertime space
  double shifted[spatial_dim +
                 2 * no_periods]; // temporal variable, projection minus centre
  /* filling the projection*/
  projection[0] = x;
  projection[1] = y;
  projection[2] = cos(heading) * speed; // velocity in the direction of x
  projection[3] = sin(heading) * speed; // velocity in the direction of y

  for (int id_n_p = 0; id_n_p < no_periods; id_n_p++) {
    projection[spatial_dim + 2 * id_n_p] =
        cos(time * 2.0 * M_PI / periodicities[id_n_p]);
    projection[spatial_dim + 2 * id_n_p + 1] =
        sin(time * 2.0 * M_PI / periodicities[id_n_p]);
  }
  /* calculate mahalanobis distance between projection and every cluster
   * centre*/
  for (int c = 0; c < no_clusters; c++) {
    for (int i = 0; i < degrees; i++) {
      shifted[i] = projection[i] - C[c][i];
    }
    distance = 0.0;
    for (int j = 0; j < degrees; j++) {
      tmp = 0.0;
      for (int i = 0; i < degrees; i++) {
        tmp = tmp + PREC[c][i][j] * shifted[i];
      }
      tmp = tmp * shifted[j];
      distance += tmp;
    }
    /*probability of occurrence from the point of view of every cluster
     * (distribution estimation)*/
    prob += gsl_cdf_chisq_Q(distance, (double)degrees) * W[c];
  }
  /*return sum of particular and weighted probabilities*/
  return prob;
}
