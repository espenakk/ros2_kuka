#pragma once
#include <Eigen/Dense>

class KalmanFilter
{
public:
    KalmanFilter();

    void init(const Eigen::Vector3d& p0, const Eigen::Vector3d& v0_guess = Eigen::Vector3d::Zero());

    void predict(double dt);
    void update(const Eigen::Vector3d& z);
    void reset();

    Eigen::Vector3d position() const { return x_.head<3>(); }
    Eigen::Vector3d velocity() const { return x_.tail<3>(); }
    Eigen::Vector3d predictFuture(double dt) const;

private:
    Eigen::VectorXd x_;      // [x y z vx vy vz]^T
    Eigen::MatrixXd P_;      // state covariance
    Eigen::MatrixXd F_;      // state transition
    Eigen::MatrixXd Q_;      // process noise
    Eigen::MatrixXd H_;      // measurement model
    Eigen::MatrixXd R_;      // measurement noise
};
