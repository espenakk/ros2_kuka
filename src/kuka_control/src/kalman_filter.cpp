#include "kalman_filter.hpp"

KalmanFilter::KalmanFilter()
{
    x_ = Eigen::VectorXd::Zero(6);
    P_ = Eigen::MatrixXd::Identity(6,6) * 1.0;

    F_ = Eigen::MatrixXd::Identity(6,6);           // will fill dt terms each predict
    Q_ = Eigen::MatrixXd::Zero(6,6);               // will set diag each predict

    H_ = Eigen::MatrixXd::Zero(3,6);
    H_.block<3,3>(0,0) = Eigen::Matrix3d::Identity();

    R_ = Eigen::MatrixXd::Identity(3,3) * 0.02;    // camera noise ≈ 2 cm
}

void KalmanFilter::init(const Eigen::Vector3d& p0, const Eigen::Vector3d& v0_guess)
{
    x_.head<3>() = p0;
    x_.tail<3>() = v0_guess;
}

void KalmanFilter::predict(double dt)
{
    // state transition for constant velocity model
    F_.setIdentity();
    F_(0,3) = dt;
    F_(1,4) = dt;
    F_(2,5) = dt;

    // simple diagonal process noise – larger on velocities
    Q_.setZero();
    Q_.block<3,3>(0,0).setIdentity() * 0.0001;   // position noise
    Q_.block<3,3>(3,3).setIdentity() * 0.10;     // velocity noise

    // gravity as control input (affects vz)
    Eigen::VectorXd u(6); u << 0,0,0, 0,0,-9.81;
    x_ = F_ * x_ + u * dt;

    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::update(const Eigen::Vector3d& z)
{
    Eigen::Vector3d y   = z - H_ * x_;
    Eigen::Matrix3d S   = H_ * P_ * H_.transpose() + R_;
    Eigen::MatrixXd K   = P_ * H_.transpose() * S.inverse();

    x_ = x_ + K * y;
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6,6);
    P_ = (I - K * H_) * P_;
}

Eigen::Vector3d KalmanFilter::predictFuture(double dt) const
{
    // integrate once more (constant-velocity + gravity)
    Eigen::Vector3d p = x_.head<3>();
    Eigen::Vector3d v = x_.tail<3>();
    Eigen::Vector3d pred;
    pred.x() = p.x() + v.x()*dt;
    pred.y() = p.y() + v.y()*dt;
    pred.z() = p.z() + v.z()*dt - 0.5*9.81*dt*dt;
    return pred;
}

void KalmanFilter::reset()
{
    x_.setZero();
    P_.setIdentity();
}
