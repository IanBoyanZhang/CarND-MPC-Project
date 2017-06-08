#ifndef MPC_TOOLS_H
#define MPC_TOOLS_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using Eigen::MatrixXd;
using Eigen::VectorXd;
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
const double Lf = 2.67;

class Tools {
public:
  Tools();
  virtual ~Tools();

  double mph_to_mps(double v);
  double polyfit(VectorXd xvals, VectorXd yvals, int order);
  double polyeval(VectorXd coeffs, double x);
  double polyeval_1st_deri(const VectorXd &coeffs, const double x);
  double get_desired_psi(const VectorXd &coeffs, const double x);
  vector<double> map2car(const double psi, const double ptsx,
                  const double ptsy, const double px, const double py);
  double get_cte(const double x0, const double y0, const VectorXd &coeffs);
  double get_epsi(const double x0, const double psi0, const VectorXd &coeffs);
};


#endif //MPC_TOOLS_H
