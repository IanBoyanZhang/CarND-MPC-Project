#ifndef MPC_TOOLS_H
#define MPC_TOOLS_H

#include <cmath>
#include "Eigen-3.3/Eigen/Core"

using Eigen::VectorXd;
using Eigen::MatrixXd;
/**
 * Evaluate a polynomial
 * @param coeffs
 * @param x
 * @return
 */
double_t polyeval(VectorXd coeffs, double_t x) {
  double_t result = 0.0;
  for (auto i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

/**
 * Fit a polynomial
 * Adapted from
 * https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
 * @param xvals
 * @param yvals
 * @param order
 * @return
 */
VectorXd polyfit(VectorXd xvals, VectorXd yvals, int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  MatrixXd A = MatrixXd::Zero(xvals.size(), order + 1);

  A.col(0) = VectorXd::Ones(xvals.size());

  // Construct a Vandermonde matrix
  for (auto i = 0; i < order; i+=1) {
    A.col(i + 1)  = A.col(i).array() * xvals.array();
  }

  auto Q = A.householderQr();
  return Q.solve(yvals);
}

#endif //MPC_TOOLS_H
