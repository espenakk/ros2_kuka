#pragma once

#include <Eigen/Dense>

class KalmanFilter
{
public:
    KalmanFilter();

    void init(const Eigen::Vector3d& initial_position, double dt);

    void predict(double dt);
    void update(const Eigen::Vector3d& measurement);

    Eigen::Vector3d getPosition() const;
    Eigen::Vector3d getVelocity() const;
    Eigen::Vector3d predictFuturePosition(double dt) const;

private:
    Eigen::VectorXd x_; // [x, y, z, vx, vy, vz]
    Eigen::MatrixXd P_; // Covariance
    Eigen::MatrixXd F_; // State transition model
    Eigen::MatrixXd Q_; // Process noise covariance
    Eigen::MatrixXd H_; // Measurement model
    Eigen::MatrixXd R_; // Measurement noise covariance
};
