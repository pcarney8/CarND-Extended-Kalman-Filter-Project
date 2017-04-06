#include "kalman_filter.h"

using namespace std;
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
    MatrixXd y = z - H_*x_;
    MatrixXd H_t = H_.transpose();
    MatrixXd S = H_*P_*H_t + R_;
    MatrixXd K = P_*H_t*S.inverse();
    MatrixXd I = MatrixXd::Identity(2, 2);
    x_ = x_ + (K*y);
    P_ = (I - (K*H_))*P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  //TODO: CONVERT MEASUREMENT
    float px = x_[0];
    float py = x_[1];
    float vx = x_[2];
    float vy = x_[3];

    float sqrtpx2py2 = sqrt(px*px + py*py);
    float pxvxpyvy = px*vx + py*vy;

    //TODO: NORMALIZE ATAN2() AND z[1]
    VectorXd h = VectorXd(3);
    h << sqrtpx2py2, atan2(py/px), pxvxpyvy/sqrtpx2py2;

    MatrixXd y = z - h;

  //H_ is Hj_ here
    MatrixXd H_t = H_.transpose();
    MatrixXd S = H_*P_*H_t + R_;
    MatrixXd K = P_*H_t*S.inverse();

    //TODO: VERIFY THIS ID MATRIX IS THE CORRECT SIZE, might be 3x3?
    MatrixXd I = MatrixXd::Identity(2, 2);

    x_ = x_ + (K*y);
    P_ = (I - (K*H_))*P_;
}
