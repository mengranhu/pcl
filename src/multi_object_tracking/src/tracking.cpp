#include "../include/tracking.h"

// #include "Eigen/Dense"
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

Tracking::Tracking()
{
    is_initialized_ = false;
    history_ = 0;
    previous_timestamp_ = 0;

    kf_.x_ = VectorXd(10);

    // state covariance matrix P
    kf_.P_ = MatrixXd(10, 10);
    kf_.P_.setConstant(1000.0);

    // measurement covariance
    kf_.R_ = MatrixXd(7, 7);
    kf_.R_.setConstant(0.1);
    // kf_.R_ << 0.0225, 0, 0, 0.0225;

    // measurement matrix
    kf_.H_ = MatrixXd(7, 10);
    kf_.H_ << 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
              0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
              0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
              0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
              0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
              0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
              0, 0, 0, 0, 0, 0, 1, 0, 0, 0;

    // the initial transition matrix F_
    kf_.F_ = MatrixXd(10, 10);
    kf_.F_ << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0,
              0, 1, 0, 0, 0, 0, 0, 0, 1, 0,
              0, 0, 1, 0, 0, 0, 0, 0, 0, 1,
              0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
              0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
              0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
              0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
              0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
              0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
              0, 0, 0, 0, 0, 0, 0, 0, 0, 1;

    // set the acceleration noise components
    noise_ax = 5;
    noise_ay = 5;
    noise_az = 1;
}

Tracking::~Tracking()
{
}

// Process only predict for next detection hasn't any object data
void Tracking::process_measurement(double predict_time_ms)
{
    if (!is_initialized_) 
    {
        cout << "no detection object predict error ！！！！！" << endl;
        return;
    }

    // compute the time elapsed between the current and previous measurements
    // dt - expressed in seconds
    float dt = predict_time_ms / 1000.0;
    previous_timestamp_ += predict_time_ms;

    // TODO: YOUR CODE HERE

    float dt_2 = dt * dt;
    float dt_3 = dt_2 * dt;
    float dt_4 = dt_3 * dt;

    // 1. Modify the F matrix so that the time is integrated
    kf_.F_(0, 7) = dt;
    kf_.F_(1, 8) = dt;
    kf_.F_(2, 9) = dt;

    // 2. Set the process covariance matrix Q
    kf_.Q_ = MatrixXd(10, 10);
    kf_.Q_ << dt_4 * noise_ax / 4, 0, 0, 0, 0, 0, 0, dt_3 * noise_ax / 2, 0, 0,
              0, dt_4 * noise_ay / 4, 0, 0, 0, 0, 0, 0, dt_3 * noise_ay / 2, 0,
              0, 0, dt_4 * noise_az / 4, 0, 0, 0, 0, 0, 0, dt_3 * noise_az / 2,
              0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0,
              0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0,
              0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0,
              0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0,
              dt_3 * noise_ax / 2, 0, 0, 0, 0, 0, 0, dt_2 * noise_ax, 0, 0,
              0, dt_3 * noise_ay / 2, 0, 0, 0, 0, 0, 0, dt_2 * noise_ay, 0,
              0, 0, dt_3 * noise_az / 2, 0, 0, 0, 0, 0, 0, dt_2 * noise_az;

    // 3. Call the Kalman Filter predict() function
    kf_.Predict();
}

// Process a single measurement
void Tracking::process_measurement(const MeasurementPackage &measurement_pack)
{
    if (!is_initialized_) {
        // cout << "Kalman Filter Initialization " << endl;

        // set the state with the initial location and zero velocity
        kf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1],
            measurement_pack.raw_measurements_[2], measurement_pack.raw_measurements_[3],
            measurement_pack.raw_measurements_[4], measurement_pack.raw_measurements_[5],
            measurement_pack.raw_measurements_[6], 0, 0, 0;

        previous_timestamp_ = measurement_pack.timestamp_;
        is_initialized_ = true;
        return;
    }

    // compute the time elapsed between the current and previous measurements
    // dt - expressed in seconds
    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000.0;
    previous_timestamp_ = measurement_pack.timestamp_;

    // TODO: YOUR CODE HERE

    float dt_2 = dt * dt;
    float dt_3 = dt_2 * dt;
    float dt_4 = dt_3 * dt;

    // 1. Modify the F matrix so that the time is integrated
    kf_.F_(0, 7) = dt;
    kf_.F_(1, 8) = dt;
    kf_.F_(2, 9) = dt;

    // 2. Set the process covariance matrix Q
    kf_.Q_ = MatrixXd(10, 10);
    kf_.Q_ << dt_4 * noise_ax / 4, 0, 0, 0, 0, 0, 0, dt_3 * noise_ax / 2, 0, 0,
              0, dt_4 * noise_ay / 4, 0, 0, 0, 0, 0, 0, dt_3 * noise_ay / 2, 0,
              0, 0, dt_4 * noise_az / 4, 0, 0, 0, 0, 0, 0, dt_3 * noise_az / 2,
              0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0,
              0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0,
              0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0,
              0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0,
              dt_3 * noise_ax / 2, 0, 0, 0, 0, 0, 0, dt_2 * noise_ax, 0, 0,
              0, dt_3 * noise_ay / 2, 0, 0, 0, 0, 0, 0, dt_2 * noise_ay, 0,
              0, 0, dt_3 * noise_az / 2, 0, 0, 0, 0, 0, 0, dt_2 * noise_az;

    // 3. Call the Kalman Filter predict() function
    kf_.Predict();
    // 4. Call the Kalman Filter update() function with the most recent raw measurements_
    kf_.Update(measurement_pack.raw_measurements_);
}
