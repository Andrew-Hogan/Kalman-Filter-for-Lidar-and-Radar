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
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  
  double rho;
  double theta;
  double rho_dot;

  if (fabs(x_(0)) < 0.0001) {
	rho = 0.001;
	theta = atan(x_(1) / 0.0001);
	rho_dot = (0.001*x_(2) + x_(1)*x_(3)) / rho;
  }
  else {
    rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
    theta = atan(x_(1) / x_(0));
    rho_dot = (x_(0)*x_(2) + x_(1)*x_(3)) / rho;
  } 
  
  //double rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
  //double theta = atan(x_(1) / x_(0));
  //double rho_dot = (x_(0)*x_(2) + x_(1)*x_(3)) / rho;

  VectorXd h = VectorXd(3);
  h << rho, theta, rho_dot;
  VectorXd y = z - h;

  while (y(1) > M_PI) {
    y(1) -= (2 * M_PI);
  }
  while (y(1) < -(M_PI)) {
    y(1) += (2 * M_PI);
  }

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
