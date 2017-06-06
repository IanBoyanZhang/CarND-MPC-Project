#ifndef MPC_H
#define MPC_H

// Reference
// https://www.coin-or.org/CppAD/Doc/cppad_eigen.hpp.htm
#include <vector>
#include <cppad/cppad.hpp>
// #include <cppad/example/cppad_eigen.hpp>
#include <limits>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

using CppAD::AD;
using Eigen::VectorXd;
using Eigen::MatrixXd;
//#include "tools.h"

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
