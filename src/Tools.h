#ifndef MPC_TOOLS_H
#define MPC_TOOLS_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;


class Tools {
public:
  Tools();
  virtual ~Tools();

  double mph_to_mps(double v);
  VectorXd polyfit(VectorXd xvals, VectorXd yvals, int order);
  double polyeval(VectorXd coeffs, double x);
  double polyeval_1st_deri(const VectorXd &coeffs, const double x);
  double get_desired_psi(const VectorXd &coeffs, const double x);
  vector<double> map2car(const double psi, const double ptsx,
                  const double ptsy, const double px, const double py);
  double get_cte(const VectorXd &coeffs);
  double get_epsi(const VectorXd &coeffs);
};


#endif //MPC_TOOLS_H
