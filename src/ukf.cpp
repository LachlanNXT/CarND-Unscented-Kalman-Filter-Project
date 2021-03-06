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
  std_a_ = 2.5;

  // Process noise standard deviation yaw acceleration in rad/s^2
  // DEFAULT VALUE VERY WRONG
  std_yawdd_ = 0.5;
  
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
  Xsig_pred_ = MatrixXd::Zero(n_x_,n_sig_);

  ///* Weights of sigma points
  weights_ = VectorXd::Zero(n_sig_);
  weights_(0) = lambda_/(lambda_+n_aug_);
  for (int i=1; i<n_sig_; i++)
  {
    double weight = 0.5/(lambda_+n_aug_);
    weights_(i) = weight;
  }

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
      x_(0) = px;
      x_(1) = py;
      cout << " RADAR " << endl;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      x_(0) = meas_package.raw_measurements_[0];
      x_(1) = meas_package.raw_measurements_[1];
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
    double delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;
    time_us_ = meas_package.timestamp_;

    // Prediction
    Prediction(delta_t);
    cout << "Prediction delta_t: " << delta_t << endl;
    cout << "x_ = \n" << x_ << endl;
    cout << "P_ = \n" << P_ << endl;

    // Update
    cout << "Update" << endl;
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
    {
      // Do UKF update for non linear Radar measurement
      cout << " RADAR " << endl;
      UpdateRadar(meas_package);
      cout << "x_ = \n" << x_ << endl;
      cout << "P_ = \n" << P_ << endl;

    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER)
    {
      // Do normal KF update for linear Laser Measurement?
      cout << " LIDAR " << endl;
      UpdateLidar(meas_package);
      cout << "x_ = \n" << x_ << endl;
      cout << "P_ = \n" << P_ << endl;

    }
  }
  return;
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
  x_aug.fill(0.0);
  x_aug.head(n_x_) = x_;
  x_aug(n_x_) = 0;
  x_aug(n_x_+1) = 0;
  // cout << "x_aug = \n" << x_aug << endl;

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_,n_x_) = P_;
  P_aug(n_x_,n_x_) = std_a_*std_a_;
  P_aug(n_x_+1,n_x_+1) = std_yawdd_*std_yawdd_;
  // cout << "P_aug = \n" << P_aug << endl;

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, n_sig_);
  Xsig_aug.fill(0.0);

  //create square root matrix
  MatrixXd A_aug = P_aug.llt().matrixL();
  
  //create augmented sigma points
  Xsig_aug.col(0) = x_aug;
  
  for (int i=0; i<n_aug_; i++)
  {
      Xsig_aug.col(i+1) = x_aug + sqrt(lambda_+n_aug_)*A_aug.col(i);
      Xsig_aug.col(i+n_aug_+1) = x_aug - sqrt(lambda_+n_aug_)*A_aug.col(i);
  }

  // cout << "Xsig_aug = \n" << Xsig_aug << endl;

  //predict sigma points
  VectorXd DeltaX = VectorXd(n_x_);
  DeltaX.fill(0.0);
  VectorXd NoiseX = VectorXd(n_x_);
  NoiseX.fill(0.0);
  VectorXd temp_X = VectorXd(n_x_);
  temp_X.fill(0.0);
  for (int i=0; i<n_sig_; i++)
  {
    const double p_x = Xsig_aug(0,i);
    const double p_y = Xsig_aug(1,i);
    const double v = Xsig_aug(2,i);
    const double yaw = Xsig_aug(3,i);
    const double yawd = Xsig_aug(4,i);
    const double nu_a = Xsig_aug(5,i);
    const double nu_yawdd = Xsig_aug(6,i);
    if (fabs(yawd) < 0.0001)
    {
      DeltaX << v*cos(yaw)*delta_t, v*sin(yaw)*delta_t, 0, 0, 0;
    }
    else
    {
      DeltaX << v/yawd*(sin(yaw+yawd*delta_t)-sin(yaw)),
                v/yawd*(-cos(yaw+yawd*delta_t)+cos(yaw)),
                0, yawd*delta_t, 0;
    }
    NoiseX << .5*delta_t*delta_t*cos(yaw)*nu_a,
              .5*delta_t*delta_t*sin(yaw)*nu_a,
              delta_t*nu_a,
              .5*delta_t*delta_t*nu_yawdd,
              delta_t*nu_yawdd;
    temp_X << p_x, p_y, v, yaw, yawd;
    Xsig_pred_.col(i) = temp_X + DeltaX + NoiseX;
  }

  cout << "Xsig_pred_ = \n" << Xsig_pred_ << endl;
  
  // Predicted Mean and Covariance
  x_.fill(0.0);
  P_.fill(0.0);
  //set weights
  
  x_ += weights_(0)*Xsig_pred_.col(0);
  for (int i=1; i<n_sig_; i++)
  {
  //predict state mean
    // std::cout << "weight\n" << weight << std::endl;
    // std::cout << "Xsig_pred column\n" << Xsig_pred_.col(i) << std::endl;
    VectorXd add = weights_(i)*Xsig_pred_.col(i);
    // std::cout << "add\n" << add << std::endl;
    x_ += add;
    // std::cout << "Predicted state\n" << x_ << std::endl;

  }
  x_(3) = atan2(sin(x_(3)),cos(x_(3)));
  //predict state covariance matrix
  for (int i=0; i<n_sig_; i++)
  {
    VectorXd x_diff = (Xsig_pred_.col(i)-x_);
    x_diff(3) = atan2(sin(x_diff(3)),cos(x_diff(3)));
    P_ += weights_(i)*x_diff*x_diff.transpose();
  }
  // cout << "weights = \n" << weights_ << endl;
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
  // Set up KF matrices
  MatrixXd H = MatrixXd(2, 5);
  H << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0;
  MatrixXd R = MatrixXd(2, 2);
  R << std_laspx_*std_laspx_, 0, 0, std_laspy_*std_laspy_;
  
  VectorXd z = meas_package.raw_measurements_;
  // Update with measurements
  VectorXd z_pred = H * x_;
	VectorXd y = z - z_pred;
  // cout << " y z z_pred \n" << y << endl << z << endl << z_pred << endl;
	MatrixXd Ht = H.transpose();
	MatrixXd S = H * P_ * Ht + R;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	MatrixXd I = MatrixXd::Identity(n_x_, n_x_);
	P_ = (I - K * H) * P_;

  cout << " LIDAR " << endl;
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
  //create matrix for sigma points in measurement space
  int n_z = 3;
  MatrixXd Zsig = MatrixXd(n_z, n_sig_);
  Zsig.fill(0.0);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  S.fill(0.0);

  //transform sigma points into measurement space
  MatrixXd R = MatrixXd(n_z,n_z);
  R << std_radr_*std_radr_, 0, 0,
       0, std_radphi_*std_radphi_, 0,
       0, 0, std_radrd_*std_radrd_;
  
  for (int i=0; i<n_sig_; i++)
  {
      float px = Xsig_pred_.col(i)(0);
      float py = Xsig_pred_.col(i)(1);
      float v = Xsig_pred_.col(i)(2);
      float yaw = Xsig_pred_.col(i)(3);
      //float yawd = Xsig_pred_.col(i)(4);
      float rho = sqrt(px*px+py*py);
      if (fabs(rho) < 0.0001)
      {
          rho = 0.0001;
      }
      
      Zsig.col(i) << rho,
                     atan2(py,px),
                     (px*cos(yaw)*v + py*sin(yaw)*v)/rho;
      //calculate mean predicted measurement
      z_pred += weights_(i)*Zsig.col(i);
      
  }
  for (int i=0; i<n_sig_; i++)
  {
      //angle normalization
      VectorXd z_diff= Zsig.col(i)-z_pred;
      while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
      while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
      MatrixXd S_add = weights_(i)*(z_diff)*(z_diff).transpose();
      // std::cout << "S_add\n" << S_add << std::endl;
      S += S_add;
      // std::cout << "S\n" << S << std::endl;
  }
  S += R;

  // std::cout << "Xsig_pred\n" << Xsig_pred_ << std::endl;
  // std::cout << "Zsig\n" << Zsig << std::endl;
  // std::cout << "Zpred\n" << z_pred << std::endl;
  // std::cout << "S\n" << S << std::endl;

  //create vector for incoming radar measurement
  VectorXd z = meas_package.raw_measurements_;
  // std::cout << "Z\n" << z << std::endl;

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);

  //calculate cross correlation matrix
  VectorXd z_diff = VectorXd(n_z);
  z_diff.fill(0.0);
  VectorXd x_diff = VectorXd(n_x_);
  x_diff.fill(0.0);

  for (int i=0; i<n_sig_; i++)
  {
      x_diff = Xsig_pred_.col(i) - x_;
      z_diff = Zsig.col(i) - z_pred;
      while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
      while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
      while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
      while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
      Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }
  //calculate Kalman gain K;
  // std::cout << "T " << std::endl << Tc << std::endl;
  // std::cout << "S " << std::endl << S << std::endl;
  MatrixXd K = Tc * S.inverse();
  // std::cout << "K " << std::endl << K << std::endl;
  //update state mean and covariance matrix
  VectorXd y = (z - z_pred);
  y(1) = atan2(sin(y(1)),cos(y(1)));
  x_ = x_ + K * y;
  x_(3) = atan2(sin(x_(3)),cos(x_(3)));
  P_ = P_ - K * S * K.transpose();

  return;
  
}
