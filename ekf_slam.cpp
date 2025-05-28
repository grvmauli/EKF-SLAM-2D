#include "ekf_slam.h"
#include "utils.h"
#include <cmath>
#include <vector>
#include <Eigen/Dense> // Required for all Eigen types


using namespace std;
using namespace Eigen;

EKFSLAM::EKFSLAM(int num_landmarks) : num_landmarks(num_landmarks) {
    int state_dim = 3 + 2 * num_landmarks;
    mu = Eigen::VectorXd::Zero(state_dim);
    sigma = Eigen::MatrixXd::Identity(state_dim, state_dim) * 0.01;
    motionNoise = Eigen::MatrixXd::Identity(3, 3) * 0.01;
}


void EKFSLAM::predict(const ControlInput& u){
    double dt = 1.0;
    double theta = mu(2);
    double dx =u.v * cos(theta)*dt;
    double dy = u.v*sin(theta)*dt;
    double dtheta = u.w*dt;

    mu(0) += dx;
    mu(1) +=dy;
    mu(2) +=dtheta;

    sigma.topLeftCorner(3,3) +=motionNoise;
}

void EKFSLAM::update(const std::vector<Observation>& observations){
    for(const auto& obs : observations){
        int idx = 3+2*obs.id;

        double dx = mu(idx) - mu(0);
        double dy = mu(idx+1) - mu(1);
        double q = dx*dx + dy*dy;

        double z_hat_range = sqrt(q);
        double z_hat_bearing = atan2(dy,dx) - mu(2);

        Eigen::Vector2d z_hat(z_hat_range,z_hat_bearing);
        Eigen::Vector2d z(obs.range, obs.bearing);

         Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, mu.size());
        H(0, 0) = -dx / z_hat_range;
        H(0, 1) = -dy / z_hat_range;
        H(1, 0) = dy / q;
        H(1, 1) = -dx / q;
        H(1, 2) = -1.0;
        H(0, idx) = dx / z_hat_range;
        H(0, idx + 1) = dy / z_hat_range;
        H(1, idx) = -dy / q;
        H(1, idx + 1) = dx / q;

        Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * 0.01;

        Eigen::MatrixXd S = H * sigma * H.transpose() + R;
        Eigen::MatrixXd K = sigma * H.transpose() * S.inverse();

        Eigen::VectorXd diff = z - z_hat;
        mu += K * diff;
        sigma = (Eigen::MatrixXd::Identity(mu.size(), mu.size()) - K * H) * sigma;

        }
}

Eigen::VectorXd EKFSLAM::getState() const {
    return mu;
}

Eigen::MatrixXd EKFSLAM::getCovariance() const {
    return sigma;
}