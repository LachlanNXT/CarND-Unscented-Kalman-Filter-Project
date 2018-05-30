#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);
  x_.fill(0.0);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);
  P_.setIdentity();

  // Process noise standard deviation longitudinal acceleration in m/s^2
  // DEFAULT VALUE VERY WRONG
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  // DEFAULT VALUE VERY WRONG
  std_yawdd_ = 30;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.

  // State, Augmented state, spreading factor
  n_x_ = 5;
  n_aug_ = 7;
  n_sig_ = 2*n_aug_ + 1;
  lambda_ = 3 - n_aug_;

  ///* predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_,n_sig_);

  ///* Weights of sigma points
  weights_ = VectorXd(n_sig_);

  ///* time when the state is true, in us
  time_us_ = 0.0;

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  // Make sure you switch between lidar and radar measurements.
  if (!is_initialized_) {
    /**
      * Initialize the state x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    time_us_ = meas_package.timestamp_;

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      float ro = meas_package.raw_measurements_[0];
      float phi = meas_package.raw_measurements_[1];
      float px = ro * cos(phi);
      float py = ro * sin(phi);
      x_ << px, py, 0, 0, 0;
      cout << " RADAR " << endl;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0, 0;
      cout << " LIDAR " << endl;
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    cout << "Initialised" << endl;
    cout << "x_ = \n" << x_ << endl;
    cout << "P_ = \n" << P_ << endl;
    return;
  }
  else
  {
    // State is initialised, normal processing to occur here
    double delta_t = meas_package.timestamp_ - time_us_;
    time_us_ = meas_package.timestamp_;
    // Prediction
    Prediction(delta_t);
    cout << "Prediction" << endl;
    cout << "x_ = \n" << x_ << endl;
    cout << "P_ = \n" << P_ << endl;
    return;
    // Update
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
    {
      // Do UKF update for non linear Radar measurement

    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER)
    {
      // Do normal KF update for linear Laser Measurement?

    }
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);
  x_aug.head(n_x_) = x_;
  x_aug(n_x_) = 0;
  x_aug(n_x_+1) = 0;

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_,n_x_) = P_;
  P_aug(n_x_,n_x_) = std_a_*std_a_;
  P_aug(n_x_+1,n_x_+1) = std_yawdd_*std_yawdd_;

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, n_sig_);

  //create square root matrix
  MatrixXd A_aug = P_aug.llt().matrixL();
  
  //create augmented sigma points
  Xsig_aug.col(0) = x_aug;
  
  for (int i=0; i<n_aug_; i++)
  {
      Xsig_aug.col(i+1) = x_aug + sqrt(lambda_+n_aug_)*A_aug.col(i);
      Xsig_aug.col(i+n_aug_+1) = x_aug - sqrt(lambda_+n_aug_)*A_aug.col(i);
  }

  //cout << "Xsig_aug = \n" << Xsig_aug << endl;

  //predict sigma points
  VectorXd DeltaX = VectorXd(n_x_);
  VectorXd NoiseX = VectorXd(n_x_);
  VectorXd x = VectorXd(n_x_);
  for (int i=0; i<n_sig_; i++)
  {
      x = Xsig_aug.col(i).head(n_x_);
      float noise_a = Xsig_aug.col(i)(5);
      float noise_b = Xsig_aug.col(i)(6);
      if (x(4) == 0)
      {
        DeltaX << x(2)*cos(x(3))*delta_t, x(2)*sin(x(3))*delta_t, 0, 0, 0;
      }
      else
      {
        DeltaX << x(2)/x(4)*(sin(x(3)+x(4)*delta_t)-sin(x(3))),
                  x(2)/x(4)*(-cos(x(3)+x(4)*delta_t)+cos(x(3))),
                  0, x(4)*delta_t, 0;
      }
      NoiseX << .5*delta_t*delta_t*cos(x(3))*noise_a,
                .5*delta_t*delta_t*sin(x(3))*noise_a,
                delta_t*noise_a,
                .5*delta_t*delta_t*noise_b,
                delta_t*noise_b;
      Xsig_pred_.col(i) = x + DeltaX + NoiseX;
  }

  //cout << "Xsig_pred_ = \n" << Xsig_pred_ << endl;
  
  // Predicted Mean and Covariance
  x_.fill(0);
  //set weights
  weights_(0) = lambda_/(lambda_+n_aug_);
  x_ += weights_(0)*Xsig_pred_.col(0);
  for (int i=1; i<n_sig_; i++)
  {
    weights_(i) = 0.5/(lambda_+n_aug_);
  //predict state mean
    x_ += weights_(i)*Xsig_pred_.col(i);
  }
  x_(3) = atan2(sin(x_(3)),cos(x_(3)));
  //predict state covariance matrix
  for (int i=0; i<n_sig_; i++)
  {
    VectorXd a = (Xsig_pred_.col(i)-x);
    a(3) = atan2(sin(a(3)),cos(a(3)));
    P_ += weights_(i)*a*a.transpose();
  }
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
}
