#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  //TODO: where is this u value coming from? are we assuming it's always zero?
    x_ = (F_*x_) + u;
    P_ = F_*P_*F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
    VectorXd z = measurements[n];
    MatrixXd y = z - H_*x_;
    MatrixXd H_t = H_.transpose();
    MatrixXd S = H_*P_*H_t + R_;
    MatrixXd K = P_*H_t*S.inverse();
    MatrixXd I = MatrixXd::Identity(2, 2);
    x_ = x_ + (K*y);
    P = (I - (K*H_))*P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
}
