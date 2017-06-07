#ifndef MPC_H
#define MPC_H

// Reference
// https://www.coin-or.org/CppAD/Doc/cppad_eigen.hpp.htm
#include <vector>
#include <cppad/cppad.hpp>
#include <limits>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

using CppAD::AD;
using Eigen::VectorXd;
using Eigen::MatrixXd;

using namespace std;

/**
 * Global variables
 */
// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
// Length from fromt to CoG that has a similar radius
const double_t Lf = 2.67;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
