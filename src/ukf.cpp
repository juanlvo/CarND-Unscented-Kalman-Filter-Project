#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;

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

  // State dimension
  n_x_ = 5;

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */

  // for verify the initialization of the object
  is_initialized_ = false;

  //verification of the time
  previous_timestamp_ = 0;

  //* radar measurement dimension
  n_zrad_ = 3;

  //* radar measurement dimension
  n_zlas_ = 2;

  //* the current NIS for radar
  NIS_radar_ = 0.0;

  //* the current NIS for laser
  NIS_laser_ = 0.0;

  //* Weights of sigma points
  weights_ = VectorXd::Zero(2*n_aug_+1);
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
	  /**
	  TODO:

	  Complete this function! Make sure you switch between lidar and radar
	  measurements.
	  */
	  if (!is_initialized_) {

		    // first measurement
		    cout << "UKF: " << endl;

		    SetIntialValues(meas_package);

			// Print the initialization results
			cout << "UKF initial: " << x_ << endl;
			// Save the initial timestamp for dt calculation
			previous_timestamp_ = meas_package.timestamp_;

			// done initializing, no need to predict or update
			is_initialized_ = true;
			return;
	  }
	  /*****************************************************************************
	   *  Prediction
	   ****************************************************************************/
	  // Calculate the timestep between measurements in seconds
	  float dt = (meas_package.timestamp_ - previous_timestamp_);
	  dt /= 1000000.0; // convert micros to s

	  Prediction(dt);

	  /*****************************************************************************
	   *  Update
	   ****************************************************************************/

	  /**
	   TODO:
	     * Use the sensor type to perform the update step.
	     * Update the state and covariance matrices.
	   */

	  if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
	    // Radar updates
		cout << "RADAR"<< endl;

	    //mean predicted measurement
	    VectorXd z_pred = VectorXd::Zero(n_zrad_);

	    //measurement covariance matrix S
	    MatrixXd S = MatrixXd::Zero(n_zrad_,n_zrad_);

	    // cross-correlation matrix Tc
	    MatrixXd Tc = MatrixXd::Zero(n_x_, n_zrad_);

	    // get predictions for x,S and Tc in RADAR space
	    PredictRadarMeasurement(z_pred, S, Tc);

	    // update the state using the RADAR measurement
	    UpdateRadar(meas_package, z_pred, Tc, S);

	  } else if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
	    // Laser updates
		cout << "LASER" << endl;

	    //mean predicted measurement
	    VectorXd z_pred = VectorXd::Zero(n_zlas_);

	    //measurement covariance matrix S
	    MatrixXd S = MatrixXd::Zero(n_zlas_,n_zlas_);

	    // cross-correlation matrix Tc
	    MatrixXd Tc = MatrixXd::Zero(n_x_, n_zlas_);

	    // get predictions for x,S and Tc in Lidar space
	    PredictLidarMeasurement(z_pred, S, Tc);

	    // update the state using the LIDAR measurement
	    UpdateLidar(meas_package, z_pred, Tc, S);

	  }

	  // update the time
	  previous_timestamp_ = meas_package.timestamp_;

	  // print the output
	  cout << "P_ = " << P_ << endl;

	  return;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

	MatrixXd Xsig_aug = MatrixXd(2*n_aug_ + 1, n_aug_);

	//create example matrix with predicted sigma points
	MatrixXd Xsig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1);

	// compute augmented sigma points
	AugmentedSigmaPoints(Xsig_aug);

	// predict augmented sigma points
	SigmaPointPrediction(Xsig_pred, Xsig_aug, delta_t);

	VectorXd x_pred = VectorXd(n_x_);
	MatrixXd P_pred = MatrixXd(n_x_, n_x_);
	PredictMeanAndCovariance(Xsig_pred, x_pred, P_pred);

	x_ = x_pred;
	P_ = P_pred;
	Xsig_pred_ = Xsig_pred;

	return;
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package, VectorXd &z_out, MatrixXd &Tc_ou, MatrixXd &S_out) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */

	//mean predicted measurement
	VectorXd z = VectorXd::Zero(n_zlas_);
	z << meas_package.raw_measurements_(0),meas_package.raw_measurements_(1);

	//Kalman gain K;
	MatrixXd K = MatrixXd::Zero(n_x_,n_zlas_);
	K = Tc_ou * S_out.inverse();

	//residual
	VectorXd z_diff = VectorXd::Zero(n_zlas_);
	z_diff = z - z_out;

	//update state mean and covariance matrix
	x_ = x_ + K * z_diff;
	P_ = P_ - K*S_out*K.transpose();
	NIS_laser_ = z_diff.transpose() * S_out.inverse() * z_diff;

	return;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement
 * @param {MeasurementPackage} meas_package The measurement at k+1
 * @param {VectorXd}           z_out prediction matrix
 * @param {MatrixXd}           Tc_ou predicted state matrix
 * @param {MatrixXd}           S_out covariance matrix
 */
void UKF::UpdateRadar(MeasurementPackage meas_package, VectorXd &z_out, MatrixXd &Tc_ou, MatrixXd &S_out) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */

	//mean predicted measurement  rho, phi, rho_dot
	VectorXd z = VectorXd::Zero(n_zrad_);
	z << meas_package.raw_measurements_(0),meas_package.raw_measurements_(1),meas_package.raw_measurements_(2);

	//Kalman gain K;
	MatrixXd K = MatrixXd::Zero(n_x_,n_zrad_);
	K = Tc_ou * S_out.inverse();

	//residual
	VectorXd z_diff = VectorXd::Zero(n_zrad_);
	z_diff = z - z_out;

	//angle normalization
	while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
	while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

	//update state mean and covariance matrix
	x_ = x_ + K * z_diff;
	P_ = P_ - K*S_out*K.transpose();
	NIS_radar_ = z_diff.transpose() * S_out.inverse() * z_diff;

	return;

}

/**
 * Predict the radar measurement
 * @param z_out {VectorXd} prediction matrix
 * @param S_out {MatrixXd} covariance matrix
 * @param Tc_ou {MatrixXd} predicted state matrix
 */
void UKF::PredictRadarMeasurement(VectorXd &z_out, MatrixXd &S_out, MatrixXd &Tc_ou) {

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_zrad_, 2 * n_aug_ + 1);

  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    // extract values for better readibility
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v  = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    // Avoid division by zero
    if(fabs(p_x) <= 0.0001){
            p_x = 0.0001;
    }
    if(fabs(p_y) <= 0.0001){
            p_y = 0.0001;
    }


    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
    Zsig(1,i) = atan2(p_y,p_x);                                 //phi
    Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
  }

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_zrad_);
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_zrad_,n_zrad_);
  S.fill(0.0);

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_zrad_);
  Tc.fill(0.0);

  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_zrad_,n_zrad_);
  R <<    std_radrd_*std_radrd_, 0, 0,
          0, std_radphi_*std_radphi_, 0,
          0, 0,std_radrd_*std_radrd_;
  S = S + R;

  //write result
  z_out = z_pred;
  S_out = S;
  Tc_ou = Tc;

  return;
}

/**
 * Predict Mean and Convariance Matrix
 * @param {MatrixXd} Xsig_pred sigma point predicted
 * @param {VectorXd} x_out state mean matrix
 * @param {MatrixXd} P_out predicted state covariance matrix
 */
void UKF::PredictMeanAndCovariance(const MatrixXd &Xsig_pred, VectorXd &x_out, MatrixXd &P_out) {

  //create vector for predicted state
  VectorXd x = VectorXd(n_x_);

  //create covariance matrix for prediction
  MatrixXd P = MatrixXd(n_x_, n_x_);

  //predicted state mean
  x.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    x = x+ weights_(i) * Xsig_pred.col(i);
  }

  //predicted state covariance matrix
  P.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points

    // state difference
    VectorXd x_diff = Xsig_pred.col(i) - x;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P = P + weights_(i) * x_diff * x_diff.transpose() ;
  }

  //write result
  x_out = x;
  P_out = P;
}

/**
 * Predict Sigma Points
 * @param {MatrixXd} Xsig_out matrix of predicted sigma points
 * @param {MatrixXd} Xsig_aug matrix with the sigma point prediction
 * @param {double}   delta_t  difference of time
 */
void UKF::SigmaPointPrediction(MatrixXd &Xsig_out, MatrixXd &Xsig_aug, double &delta_t) {

  //create matrix with predicted sigma points as columns
  MatrixXd Xsig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1);

  //predict sigma points
  for (int i = 0; i< 2*n_aug_+1; i++)
  {
    //extract values for better readability
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    }
    else {
        px_p = p_x + v*delta_t*cos(yaw);
        py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    //add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    //write predicted sigma point into right column
    Xsig_pred(0,i) = px_p;
    Xsig_pred(1,i) = py_p;
    Xsig_pred(2,i) = v_p;
    Xsig_pred(3,i) = yaw_p;
    Xsig_pred(4,i) = yawd_p;
  }

  //write result
  Xsig_out = Xsig_pred;

  return;

}

/**
 * Method Augmentation of Sigma Points
 * @param Xsig_out augmented sigma points
 */
void UKF::AugmentedSigmaPoints(MatrixXd &Xsig_out) {

  //set state dimension
  int n_x = 5;

  //set augmented dimension
  int n_aug = 7;

  //Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a = 0.2;

  //Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd = 0.2;

  //define spreading parameter
  double lambda = 3 - n_aug;

  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug, 2 * n_aug + 1);

  //create augmented mean state
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  //create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a*std_a;
  P_aug(6,6) = std_yawdd*std_yawdd;

  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  //create augmented sigma points
  Xsig_aug.col(0)  = x_aug;
  for (int i = 0; i< n_aug; i++)
  {
    Xsig_aug.col(i+1)       = x_aug + sqrt(lambda+n_aug) * L.col(i);
    Xsig_aug.col(i+1+n_aug) = x_aug - sqrt(lambda+n_aug) * L.col(i);
  }

  //print result
  std::cout << "Xsig_aug = " << std::endl << Xsig_aug << std::endl;

  //write result
  Xsig_out = Xsig_aug;

  return;

}

/**
 * SetIntialValues program for initialize the object
 * @param {MeasurementPackage} meas_package, initialize the initial values of the covariance matrix
 */
void UKF::SetIntialValues(MeasurementPackage meas_package) {

	  // Initial state Covariance Matrix P
	  P_ = MatrixXd(5, 5);
	  P_ <<   1, 0, 0, 0, 0,
	          0, 1, 0, 0, 0,
	          0, 0, 1, 0, 0,
	          0, 0, 0, 1, 0,
	          0, 0, 0, 0, 1;


	if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
	  /**
		 Convert radar from polar to Cartesian coordinates and initialize state.
	  */
	  float rho = meas_package.raw_measurements_[0];
	  float phi = meas_package.raw_measurements_[1];
	  float rho_prime = meas_package.raw_measurements_[2];

	  //Conversion from polar to Cartesian coordinates and initialize it

	  float x = rho * cos(phi);
	  float y = rho * sin(phi);
	  float vx = rho_prime * cos(phi);
	  float vy = rho_prime * sin(phi);

	  x_ << x, y, sqrt(vx*vx +vy*vy),0,0;

	} else if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {

		x_ << meas_package.raw_measurements_(0), meas_package.raw_measurements_(1), 0, 0, 0;

	}

	// Print the initialization results
	cout << "UKF initial: " << x_ << endl;

	return;
}

/**
 * PredictLidarMeasurement method for predict the measurement for Lidar
 * @param {VectorXd} z_out  Prediction matrix
 * @param {MatrixXd} S_out  Covariance matrix
 * @param {MatrixXd} Tc_out Predicted state
 */
void UKF::PredictLidarMeasurement(VectorXd &z_out, MatrixXd &S_out, MatrixXd &Tc_out) {

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_zlas_, 2 * n_aug_ + 1);

  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 sigma points

    // measurement model
    Zsig(0,i) = Xsig_pred_(0,i);          //px
    Zsig(1,i) = Xsig_pred_(1,i);          //py

  }

  //mean predicted measurement
  static VectorXd z_pred = VectorXd(n_zlas_);
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //measurement covariance matrix S
  static MatrixXd S = MatrixXd(n_zlas_,n_zlas_);
  S.fill(0.0);

  //create matrix for cross correlation Tc
  static MatrixXd Tc = MatrixXd(n_x_, n_zlas_);
  Tc.fill(0.0);

  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    S = S + weights_(i) * z_diff * z_diff.transpose();

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();

  }

  //add measurement noise covariance matrix
  static MatrixXd R = MatrixXd(n_zlas_,n_zlas_);
  R <<    pow(std_laspx_,2), 0,
          0, pow(std_laspy_,2);
  S = S + R;

  //write result
  z_out = z_pred;
  S_out = S;
  Tc_out = Tc;

  return;

}
