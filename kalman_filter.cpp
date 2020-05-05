#include <math.h>
#include "kalman_filter.h"

#define PI 3.14159265

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

/* I don't need to Init the Kalman Filter class so I comment it ***
 *
void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in; */

// I need to pass the Matrices H & R to the Update function
// depending on the type of sensor
void KalmanFilter::Init(MatrixXd &H_in, MatrixXd &R_in) {
  H_ = H_in;
  R_ = R_in;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  // transform cartesian to polar coordinates
  double pol0 = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
  double pol1 = atan(x_(1)/x_(0));
  double pol2 = (x_(0)*x_(2) + x_(1)*x_(3))/pol0;
  // create predicted location vector
  VectorXd z_pred(3);
  z_pred << pol0, pol1, pol2;
  // calculate the error y (measurement - predicted)
  VectorXd y = z - z_pred;
  // make sure that -pi < y(1) < pi
  while (y(1) > PI) {
    y(1) -= 2*PI;
  }
  while (y(1) < PI) {
    y(1) += 2*PI;
  }

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
