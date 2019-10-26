#pragma once

#include <string>
#include <vector>

class WHyTeMap {

public:
  WHyTeMap();
  ~WHyTeMap();

  void readFromXML(const std::string &fileName);

  double getLikelihood(double time, double x, double y, double heading,
                       double speed);

private:
  long no_clusters;
  long no_periods;

  std::vector<double> periodicities;

  std::vector<double> W;

  std::vector<std::vector<double>> C;

  std::vector<std::vector<std::vector<double>>> PREC;
};
