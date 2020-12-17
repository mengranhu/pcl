#ifndef TRACKING_H_
#define TRACKING_H_

#include "kalman_filter.h"
#include "measurement_package.h"

class Tracking
{
public:
  Tracking();
  virtual ~Tracking();
  void process_measurement(double predict_time);
  void process_measurement(const MeasurementPackage &measurement_pack);
  KalmanFilter kf_;
  unsigned int history_; // to statistics the time that calculates tacker new or delete, init it equal 0;
                         // unmatching 2 times need to delete

private:
  bool is_initialized_;
  int64_t previous_timestamp_;

  // acceleration noise components
  float noise_ax;
  float noise_ay;
  float noise_az;
};

#endif // TRACKING_H_
