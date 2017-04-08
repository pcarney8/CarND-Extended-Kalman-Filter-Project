#include "kalman_filter.h"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  cout << "Initializing matrices" << endl;
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
  I_ = MatrixXd::Identity(4, 4);
}

void KalmanFilter::Predict() {
  /**
    * predict the state
  */
    cout << "Predict..." << endl;
    x_ = (F_*x_);
    P_ = F_*P_*F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
    * update the state by using Kalman Filter equations
  */
    cout << "Update..." << endl;

    MatrixXd y = z - H_*x_; //2x1
    MatrixXd H_t = H_.transpose(); //4x2
    MatrixXd S = H_*P_*H_t + R_; //2x2
    MatrixXd K = P_*H_t*S.inverse(); //4x2
    x_ = x_ + (K*y); //4x1
    P_ = (I_ - (K*H_))*P_; //4x4

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
    * update the state by using Extended Kalman Filter equations
  */
    cout << "Update EKF..." << endl;

    //Get x' values
    float px = x_[0];
    float py = x_[1];
    float vx = x_[2];
    float vy = x_[3];

    float sqrtpx2py2 = sqrt(px*px + py*py);
    float pxvxpyvy = px*vx + py*vy;

    cout << "Calculate h(x')" << endl;
    VectorXd h = VectorXd(3);
    h << sqrtpx2py2, atan2(py,px), pxvxpyvy/sqrtpx2py2;

    cout << "update state using Extndend Kalman Filter equations" << endl;
    VectorXd y = z - h;

    //Normalize y[1]
    cout << "before normalize angle: " << y[1] << endl;
    y[1] = tools.NormalizeAngle(y[1]);
    cout << "after normalize angle: " << y[1] << endl;
  
    //H_ is Hj_ here
    MatrixXd H_t = H_.transpose();
    MatrixXd S = H_*P_*H_t + R_;
    MatrixXd K = P_*H_t*S.inverse();

    x_ = x_ + (K*y);
    P_ = (I_ - (K*H_))*P_;
}
