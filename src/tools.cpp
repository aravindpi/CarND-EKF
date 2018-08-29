#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

  // initialize rmse
  VectorXd  rmse(4);
  rmse << 0, 0, 0, 0;
  
  // check estimations first
  size_t vecSize = estimations.size();
  if ((vecSize == 0) || (vecSize != ground_truth.size()) ) {
    std::cout << "Invalid RMSE: " << std::endl;
    return rmse;
  }

  // Loop and find the square of differences
  for (size_t idx = 0; idx < vecSize; idx++) {
    VectorXd tmp = estimations[idx] -ground_truth[idx];
    tmp = tmp.array() * tmp.array();
    rmse  += tmp;
  }

  // calculate sqrt of mean
  rmse = rmse/vecSize;
  rmse = rmse.array().sqrt();

  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {

  MatrixXd Hj(3, 4);
  Hj = MatrixXd::Zero(3,4);

  // state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // calcuate c1, c2, c3
  float c1 = pow(px, 2) + pow(py, 2);
  if (c1 == 0) {
    std::cout << "Divide by zero error" << std::endl;
    return Hj;
  }

  float c2 = sqrt(c1);
  float c3 = c1*c2;

  //
  Hj << (px/c2), (py/c2), 0, 0,
    -(py/c1), (px/c1), 0, 0,
    py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

  return Hj;
}
