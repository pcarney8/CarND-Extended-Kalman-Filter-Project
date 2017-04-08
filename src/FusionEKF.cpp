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
  // create covariance matrices

  x_ = VectorXd(4);
  P_ = MatrixXd(4,4);
  F_ = MatrixXd(4,4);
  Q_ = MatrixXd(4,4);

  F_ << 1, 0, 1, 0,
		0, 1, 0, 1,
		0, 0, 1, 0,
		0, 0, 0, 1;
		
  P_ << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1000, 0,
		0, 0, 0, 1000;
		
  //Initialize the H matrix for laser
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  //Initialize the H jacobian
  //TODO: VERIFY THAT 1'S ARE OK HERE
  Hj_ << 1, 1, 0, 0,
         1, 1, 0, 0,
         1, 1, 1, 1;

  //set acceleration noise
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
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */

    cout << "Initializing EKF..." << endl;

    // first measurement
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    previous_timestamp_ = measurement_pack.timestamp_;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      cout << "first measurement is RADAR" << endl;

      ekf_.Init(x_, P_, F_, Hj_, R_radar_, Q_);

      float rho = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];
      float rho_dot = measurement_pack.raw_measurements_[2];

      cout << "convert to cartesian" << endl;
      float px = rho * cos(phi);
      float py = rho * sin(phi);

      ekf_.x_ << px, py, 0, 0;

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      cout << "first measurement is LIDAR" << endl;

      ekf_.Init(x_, P_, F_, H_laser_, R_laser_, Q_);

      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    }

    cout << "px: " << ekf_.x_[0] << endl;
    cout << "py: " << ekf_.x_[1] << endl;
    cout << "vx: " << ekf_.x_[2] << endl;
    cout << "vy: " << ekf_.x_[3] << endl;

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

  cout << "compute the time elapsed between the current and previous measurements" << endl;
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;

  cout << "update the state transition matrix F with new elapsed time" << endl;
  ekf_.F_(0,2) = dt;
  ekf_.F_(1,3) = dt;

  cout << "update process covariance matrix Q" << endl;
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
			   0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
			   dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
			   0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

  ekf_.Predict();
  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
  
  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    cout << "Radar updates" << endl;
    //TODO: DO I NEED TO CHANGE THE SIZE OF THESE FIRST?
//    ekf_.R_ = MatrixXd(3, 3);
//    ekf_.H_ = MatrixXd(3, 4);

    //TODO: CHECK PX*PX + PY*PY CLOSE TO ZERO?
    //doing this in the calculate jacobian method,
    //does it need to be here too?

    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.R_ = R_radar_;
    ekf_.H_ = Hj_;

    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    cout << "Laser updates" << endl;
    ekf_.R_ = R_laser_;
    ekf_.H_ = H_laser_;

    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
