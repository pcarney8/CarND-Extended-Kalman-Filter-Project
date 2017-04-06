#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */

  //Initialize the H matrix for laser
  H_laser_ << 1, 0, 0, 0
              0, 1, 0, 0;

  //Initialize the H jacobian
  //TODO: VERIFY THAT 1'S ARE OK HERE
  Hj_ << 1, 1, 0, 0,
         1, 1, 0, 0,
         1, 1, 1, 1;

  //TODO: SET THE PROCESS AND MEASUREMENT NOISES
  //does this go here?
  noise_ax = 9.0;
  noise_ay = 9.0;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    cout << "Initialization" << endl;

    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      *
      * Create the covariance matrix. <-- are we talking about Q here?,
      * and why are we creating it in the initialize block? this might just be initializing
      * the ekf_.Init(...) function
      *
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */

    // first measurement
    cout << "EKF: " << endl;

    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    previous_timestamp_ = measurement_pack.timestamp_;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      float ro = measurement_pack[0];
      float phi = measurement_pack[1];

      //TODO: do i need to limit this between -pi and pi here?
      float px = ro * cos(phi);
      float py = ro * sin(phi);

      //TODO: do i need to add velocity? probably....


      ekf_.x_ << px, py, 0, 0;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      //Laser doesn't have velocity, only position?
      //velocity is zero because it's the first measurement and we don't have another time value, right?
      ekf_.x_ << measurement_pack[0], measurement_pack[1], 0, 0;
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  //compute the time elapsed between the current and previous measurements
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;

  //1. Modify the F matrix so that the time is integrated
  kf_.F_(0,2) = dt;
  kf_.F_(1,3) = dt;
  //2. Set the process covariance matrix Q
  kf_.Q_ = MatrixXd(4,4);
  float dt2 = dt * dt;
  float dt3 = dt2 * dt * 0.5;
  float dt4 = dt3 * dt * 0.5;

  kf_.Q_ << dt4 * noise_ax, 0, dt3 * noise_ax, 0,
            0, dt4 * noise_ay, 0, dt3 * noise_ay,
            dt3 * noise_ax, 0, dt2 * noise_ax, 0,
            0, dt3 * noise_ay, 0, dt2 * noise_ay;

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    ekf_.UpdateEKF(measurement_pack);
  } else {
    // Laser updates
    ekf_.Update(measurement_pack);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
