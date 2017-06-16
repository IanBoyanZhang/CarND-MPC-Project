#ifndef MPC_H
#define MPC_H

// Reference
// https://www.coin-or.org/CppAD/Doc/cppad_eigen.hpp.htm
#include <vector>
#include <cppad/cppad.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;
using Eigen::VectorXd;
using Eigen::MatrixXd;

const double Lf = 2.67;
const double latency = 0.1;

class MPC {
public:
  MPC();

  virtual ~MPC();

  double steer_value;
  double throttle_value;

  std::vector<double> predicted_x_vals;
  std::vector<double> predicted_y_vals;

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuations.
  void Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
