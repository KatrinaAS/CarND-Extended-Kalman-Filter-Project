#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
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

  ekf_.x_ = VectorXd(4);
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
	  0, 1, 0, 1,
	  0, 0, 1, 0,
	  0, 0, 0, 1;

// measurement matrix
  H_laser_ = MatrixXd(2, 4);
  H_laser_ << 1, 0, 0, 0,
	  0, 1, 0, 0;

// state covariance matrix P
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
	  0, 1, 0, 0,
	  0, 0, 1000, 0,
	  0, 0, 0, 1000;

  noise_ax = 9;
  noise_ay = 9;

}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {
}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
/**
 * Initialization
 */
  if (!is_initialized_) {


// first measurement
	cout << "EKF: " << endl;
	ekf_.x_ = VectorXd(4);
	ekf_.x_ << 1, 1, 1, 1;

	if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
	  float rho = measurement_pack.raw_measurements_(0);
	  float phi = measurement_pack.raw_measurements_(1);
	  float px = cos(phi) * rho;
	  float py = sin(phi) * rho;
// 0 for vx and vy as while we could calculate rhodot, it wouldn't contain useful information
	  ekf_.x_ << px,
		  py,
		  0,
		  0;
	} else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
	  ekf_.x_ << measurement_pack.raw_measurements_[0],
		  measurement_pack.raw_measurements_[1],
		  0,
		  0;
	}

	previous_timestamp_ = measurement_pack.timestamp_;
// done initializing, no need to predict or update
	is_initialized_ = true;
	return;
  }

/**
 * Prediction
 */

  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

// Setting to zero, as otherwise undefined values cause random errors
  ekf_.Q_ = MatrixXd(4, 4).setZero();
  ekf_.Q_(0, 0) = (pow(dt, 4) / 4) * noise_ax;
  ekf_.Q_(0, 2) = (pow(dt, 3) / 2) * noise_ax;
  ekf_.Q_(1, 1) = (pow(dt, 4) / 4) * noise_ay;
  ekf_.Q_(1, 3) = (pow(dt, 3) / 2) * noise_ay;
  ekf_.Q_(2, 0) = (pow(dt, 3) / 2) * noise_ax;
  ekf_.Q_(2, 2) = (pow(dt, 2)) * noise_ax;
  ekf_.Q_(3, 1) = (pow(dt, 3) / 2) * noise_ay;
  ekf_.Q_(3, 3) = (pow(dt, 2)) * noise_ay;

  ekf_.Predict();


/**
 * Update
 */


  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
	ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
	ekf_.R_ = R_radar_;
	ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
	ekf_.H_ = H_laser_;
	ekf_.R_ = R_laser_;

	ekf_.Update(measurement_pack.raw_measurements_);

  }

// print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
