#include "kalman_filter.hpp"

KalmanFilter::KalmanFilter()
{
    x_ = Eigen::VectorXd::Zero(6); // [x, y, z, vx, vy, vz]
    P_ = Eigen::MatrixXd::Identity(6, 6) * 1.0;
    F_ = Eigen::MatrixXd::Identity(6, 6);
    Q_ = Eigen::MatrixXd::Identity(6, 6) * 0.01;
    H_ = Eigen::MatrixXd::Zero(3, 6);
    H_.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
    R_ = Eigen::MatrixXd::Identity(3, 3) * 0.05;
}

void KalmanFilter::init(const Eigen::Vector3d& initial_position, double dt)
{
    x_.head<3>() = initial_position;
    x_.tail<3>() = Eigen::Vector3d::Zero(); // Assume zero initial velocity
}

void KalmanFilter::predict(double dt)
{
    F_.setIdentity();
    F_(0,3) = dt;
    F_(1,4) = dt;
    F_(2,5) = dt;

    x_ = F_ * x_;
    x_(5) -= 9.81 * dt; // gravity effect on vz

    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::update(const Eigen::Vector3d& z)
{
    Eigen::Vector3d y = z - H_ * x_;
    Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
    Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();

    x_ = x_ + K * y;
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);
    P_ = (I - K * H_) * P_;
}

Eigen::Vector3d KalmanFilter::getPosition() const
{
    return x_.head<3>();
}

Eigen::Vector3d KalmanFilter::getVelocity() const
{
    return x_.tail<3>();
}

Eigen::Vector3d KalmanFilter::predictFuturePosition(double dt) const
{
    Eigen::Vector3d pos = getPosition();
    Eigen::Vector3d vel = getVelocity();
    Eigen::Vector3d predicted_pos;

    predicted_pos.x() = pos.x() + vel.x() * dt;
    predicted_pos.y() = pos.y() + vel.y() * dt;
    predicted_pos.z() = pos.z() + vel.z() * dt - 0.5 * 9.81 * dt * dt; // gravity

    return predicted_pos;
}
