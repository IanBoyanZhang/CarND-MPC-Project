#include "MPC.h"
#include "Tools.h"
#include <cppad/ipopt/solve.hpp>

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
// Length from front to CoG that has a similar radius
size_t N = 16;
double dt = 0.016;

// Vehicle configuration variables
double steering_radius_lb = -0.436332;
double steering_radius_ub = 0.436332;

double ref_cte = 0;
double ref_epsi = 0;
// Target speed
double ref_v = 0;

size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

double w_cost_ref_cte;
double w_cost_ref_epsi;
double w_cost_ref_v;
double w_cost_ref_val_steering;
double w_cost_ref_val_throttle;
double w_cost_ref_seq_steering;
double w_cost_ref_seq_throttle;

Tools tools;

class FG_eval {
 public:
  // Polynomial coefficients
  Eigen::VectorXd coeffs;

  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // `fg` a vector of the cost and constraints, `vars` is a vector of variable values (state & actuators)

    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.

    // https://www.coin-or.org/CppAD/Doc/ipopt_solve.htm
    // Setting up cost function
    // First element of `fg` is cost
    fg[0] = 0;

    // The part of the cost based on the reference state
    // Least square error in quadratic form?

    // The part of the cost based on the reference state.
    for (int i = 0; i < N; i++) {
      fg[0] += w_cost_ref_cte * CppAD::pow(vars[cte_start + i] - ref_cte, 2);
      fg[0] += w_cost_ref_epsi * CppAD::pow(vars[epsi_start + i] - ref_epsi, 2);
      fg[0] += w_cost_ref_v * CppAD::pow(vars[v_start + i] - ref_v, 2);
    }

    // Minimize the use of actuators.
    for (int i = 0; i < N - 1; i++) {
      fg[0] += w_cost_ref_val_steering * CppAD::pow(vars[delta_start + i], 2);
      fg[0] += w_cost_ref_val_throttle * CppAD::pow(vars[a_start + i], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (int i = 0; i < N - 2; i++) {
      fg[0] += w_cost_ref_seq_steering * CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
      fg[0] += w_cost_ref_seq_throttle * CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
    }

    // Setup Constraints
    // g(x) model constraints

    // Initial constraints
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // The rest of the constraints
    for (int i = 0; i < N - 1; i++) {
      // The state at time t+1 .
      AD<double> x1 = vars[x_start + i + 1];
      AD<double> y1 = vars[y_start + i + 1];
      AD<double> psi1 = vars[psi_start + i + 1];
      AD<double> v1 = vars[v_start + i + 1];
      AD<double> cte1 = vars[cte_start + i + 1];
      AD<double> epsi1 = vars[epsi_start + i + 1];

      // The state at time t.
      AD<double> x0 = vars[x_start + i];
      AD<double> y0 = vars[y_start + i];
      AD<double> psi0 = vars[psi_start + i];
      AD<double> v0 = vars[v_start + i];
      AD<double> cte0 = vars[cte_start + i];
      AD<double> epsi0 = vars[epsi_start + i];

      // Only consider the actuation at time t.
      AD<double> delta0 = vars[delta_start + i];
      AD<double> a0 = vars[a_start + i];

      AD<double> f0 = coeffs[0] +
                      coeffs[1] * x0 +
                      coeffs[2] * CppAD::pow(x0, 2) +
                      coeffs[3] * CppAD::pow(x0, 3);
      AD<double> f_t0_rate_of_change = coeffs[1] +
                                      (2 * coeffs[2] * x0) +
                                      (3 * coeffs[3] * CppAD::pow(x0, 2));
      // Second order
      AD<double> psides0 = CppAD::atan(f_t0_rate_of_change);

      // Here's `x` to get you started.
      // The idea here is to constraint this value to be 0.
      //
      // Recall the equations for the model:
      // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      // v_[t+1] = v[t] + a[t] * dt
      // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
      fg[1 + x_start + i + 1] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + i + 1] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + i + 1] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
      fg[1 + v_start + i + 1] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + i + 1] =
          cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + i + 1] =
          epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC(vector<double> &hyper_params) {
  /*************************************************************************
   * Cost weights hyper parameters
   *************************************************************************/
  w_cost_ref_cte = hyper_params[0];
  w_cost_ref_epsi = hyper_params[1];
  w_cost_ref_v = hyper_params[2];
  w_cost_ref_val_steering = hyper_params[3];
  w_cost_ref_val_throttle = hyper_params[4];
  w_cost_ref_seq_steering = hyper_params[5];
  w_cost_ref_seq_throttle = hyper_params[6];
  ref_v = tools.mph_to_mps(hyper_params[7]);
}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:

  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  // Number of independent variables
  size_t n_vars = N * (4 + 2) + (N - 1) * 2;
  // Set the number of constraints
  size_t n_constraints = N * (4 + 2);

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (auto i = 0; i < n_vars; i++) {
    vars[i] = 0.0;
  }

  // Set the initial variable values
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;

  // Set the initial variable values
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Set lower and upper limits for variables.
  // Set all non-actuators upper and lower limits
  // to the max negative and positive values
  for (auto i = 0; i < delta_start; i+=1) {
    vars_lowerbound[i] = std::numeric_limits<double>::lowest();
    vars_upperbound[i] = std::numeric_limits<double>::max();
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians)
  // NOTE: Feel free to change this to something else.
  for (auto i = delta_start; i < a_start; i+=1){
    vars_lowerbound[i] = steering_radius_lb;
    vars_upperbound[i] = steering_radius_ub;
  }

  // Acceleration/deacceleration upper and lower limits.
  // NOTE: Feel free to change this to something else.
  for (auto i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  /**
   * Lower and upper limits for the constraints
   * Should be 0 besides initial state.
   */

  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);

  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);
  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level 0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options,
      vars,
      vars_lowerbound,
      vars_upperbound,
      constraints_lowerbound,
      constraints_upperbound,
      fg_eval,
      solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.

  vector<double> results;

  /*results.push_back(solution.x[psi_start + 1]);
  results.push_back(solution.x[v_start + 1]);
  results.push_back(solution.x[cte_start + 1]);
  results.push_back(solution.x[epsi_start + 1]);*/
  steer_value = solution.x[delta_start];
  throttle_value = solution.x[a_start];

  for (int i = 0; i < N; i+=1) {
    results.push_back(solution.x[x_start + i]);
    results.push_back(solution.x[y_start + i]);
  }
  return results;
}
