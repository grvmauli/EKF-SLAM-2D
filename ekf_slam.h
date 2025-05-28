// ekf_slam.h
#pragma once

#include <Eigen/Dense>
#include <vector>
#include "utils.h"  // Defines ControlInput, Observation, etc.

class EKFSLAM {
public:
    EKFSLAM(int num_landmarks);

    void predict(const ControlInput& u);
    void update(const std::vector<Observation>& observations);

    Eigen::VectorXd getState() const;
    Eigen::MatrixXd getCovariance() const;

private:
    int num_landmarks;
    Eigen::VectorXd mu;      // State mean
    Eigen::MatrixXd sigma;   // Covariance matrix
    Eigen::MatrixXd motionNoise;
};
