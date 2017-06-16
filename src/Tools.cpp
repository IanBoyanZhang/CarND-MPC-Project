#include "Tools.h"

Tools::Tools() {

}

Tools::~Tools(){}

double Tools::mph_to_mps(double v) {
  return v * 1609/3600;
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
VectorXd Tools::polyfit(VectorXd xvals, VectorXd yvals, int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  MatrixXd A = MatrixXd::Zero(xvals.size(), order + 1);

  // Construct a Vandermonde matrix
  A.col(0) = VectorXd::Ones(xvals.size());
  for (auto i = 0; i < order; i+=1) {
    A.col(i + 1)  = A.col(i).array() * xvals.array();
  }

  auto Q = A.householderQr();
  return Q.solve(yvals);
}


// Evaluate a polynomial.
double Tools::polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

/**
 * Evaluate first order derivative of fitted polynomial
 * TODO: Rewrite using Vandermonde matrix?
 * @param coeffs
 * @param x
 * @return
 */
double Tools::polyeval_1st_deri(const VectorXd &coeffs, const double x) {
  double result = 0.0;
  for (auto i = coeffs.size() - 1; i > 0; i -= 1) {
    result += i * coeffs[i] * pow(x, i - 1);
  }
  return result;
}

double Tools::get_desired_psi(const VectorXd &coeffs, const double x) {
  double d_fx = polyeval_1st_deri(coeffs, x);
  return atan(d_fx);
}

// Coordinate transformation
/**
 *  Transform navigation points in navigation/map coordinates to
 *  car/vehicle
 * @param psi
 * @param ptsx
 * @param ptsy
 * @param px
 * @return nav_in_car_x, nav_in_car_y
 */
vector<double> Tools::map2car(const double psi, const double ptsx,
                   const double ptsy, const double px, const double py) {
  double x = ptsx - px;
  double y = ptsy - py;

  vector<double> result;
  result.push_back(x * cos(-psi) - y * sin(-psi));
  result.push_back(x * sin(-psi) + y * cos(-psi));
  return result;
}

/**
 * @param x0
 * @param y0
 * @param coeffs
 * @return
 */
double Tools::get_cte(const double x0, const double y0, const VectorXd &coeffs) {
  //return polyeval(coeffs, x0) - y0;
  return coeffs[0];
}

/**
 *
 * @param x0
 * @param psi0
 * @param coeffs
 * @return
 */
double Tools::get_epsi(const double x0, const double psi0, const VectorXd &coeffs) {
  //return psi0 - polyeval_1st_deri(coeffs, x0);
  return -atan(coeffs[1]);
}
