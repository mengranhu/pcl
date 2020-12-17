#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

// #include "Eigen/Dense"
#include <eigen3/Eigen/Dense>

class MeasurementPackage
{
public:
  Eigen::VectorXd raw_measurements_;

  int64_t timestamp_;
};

#endif // MEASUREMENT_PACKAGE_H_
