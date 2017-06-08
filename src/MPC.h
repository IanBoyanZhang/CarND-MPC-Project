#ifndef MPC_H
#define MPC_H

// Reference
// https://www.coin-or.org/CppAD/Doc/cppad_eigen.hpp.htm
#include <vector>
#include <cppad/cppad.hpp>
#include <limits>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "TOOLS.h"

using CppAD::AD;
using Eigen::VectorXd;
using Eigen::MatrixXd;

using namespace std;

class MPC {
public:
  MPC();
  virtual ~MPC();

  double steer_value;
  double throttle_value;

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuations.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
