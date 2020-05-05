#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

using namespace std;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if (!estimations.size()){
    cout << "Estimation Vector is empty! Return RMSE =" << endl;
    return rmse;
  }
  else if (estimations.size() != ground_truth.size()){
    cout << "Estimation Vector and Ground Truth don't match!" << endl;
    cout << "Return RMSE =" << endl;
    return rmse;
  }

  // accumulate squared residuals
  for (int i=0; i < estimations.size(); ++i) {
    // ... your code here
    VectorXd diff(4);
    VectorXd diff_2(4);

    diff = estimations[i] - ground_truth[i];
    diff_2 = diff.array()*diff.array();
    rmse = rmse + diff_2;
  }

  // calculate the mean
  rmse = rmse.array()/estimations.size();
  // calculate the squared root
  rmse = rmse.array().sqrt();

  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO: Calculate a Jacobian here.
   */
  MatrixXd Hj(3,4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  double px_2 = px*px;
  double py_2 = py*py;
  double densq = px_2 + py_2;
  double denrt = sqrt(densq);
  double den_32 = denrt*densq;

  // check division by zero
  if (!px && !py){
    cout << "Error! Division by 0!" << endl;
    return Hj;
  }

  // compute the Jacobian matrix
  Hj << px/denrt, py/denrt, 0, 0,
        -py/densq, px/densq, 0, 0,
        py*(vx*py - vy*px)/den_32, px*(vy*px - vx*py)/den_32, px/denrt, py/denrt;

  return Hj;
}
