#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
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
double polyeval_1st_deri(const VectorXd &coeffs, const double x) {
  double result = 0.0;
  for (int i = coeffs.size() - 1; i > 0; i -= 1) {
    result += i * coeffs[i] * pow(x, i - 1);
  }
  return result;
}

double get_desired_psi(const VectorXd &coeffs, const double x) {
  double d_fx = polyeval_1st_deri(coeffs, x);
  return atan(d_fx);
}


/**
 * Evaluate a polynomial
 * @param coeffs
 * @param x
 * @return
 */
/*
double_t polyeval(VectorXd coeffs, double_t x) {
  double_t result = 0.0;
  for (auto i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}*/

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  return Q.solve(yvals);
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
/*
VectorXd polyfit(VectorXd xvals, VectorXd yvals, int order) {
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
}*/

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
vector<double> map2car(const double psi, const double ptsx,
                   const double ptsy, const double px, const double py) {
  double x = ptsx - px;
  double y = ptsy - py;

  vector<double> result;
  result.push_back(x * cos(-psi) - y * sin(-psi));
  result.push_back(x * sin(-psi) + y * cos(-psi));
  return result;
}

/**
 *
 * @param x0
 * @param y0
 * @param coeffs
 * @return
 */
double get_cte(const double x0, const double y0, const VectorXd &coeffs) {
  return polyeval(coeffs, x0) - y0;
}

/**
 *
 * @param x0
 * @param psi0
 * @param coeffs
 * @return
 */
double get_epsi(const double x0, const double psi0, const VectorXd &coeffs) {
  return psi0 - polyeval_1st_deri(coeffs, x0);
}

/**
 * Progress state variables after time dt
 * Based on vehicle kinematics model
 * @param x
 * @param y
 * @param psi
 * @param v
 * @param cte
 * @param epsi
 * @param delta
 * @param a
 * @param dt
 */
void progress_state(double *x, double *y, double *psi, double *v,
                    double *cte, double *epsi, const double delta,
                    const double a, const double dt) {

  *x = *x + *v * cos(*psi) * dt;
  *y = *y + *v * sin(*psi) * dt;
  *psi = *psi + *v * delta / Lf * dt;
  *cte = *cte + *v * sin(*epsi) * dt;
  *epsi = *epsi + *v * delta/ Lf * dt;
  *v = *v + a * delta * dt;
}

double mph_to_mps(const double v) {
  return v * 0.44704;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          /**
           * Accounting actuators delay
           */
          double steer_value = j[1]["steering_angle"];
          double throttle_value = j[1]["throttle"];

//          Server usually provides 6 navigation points
//          Can be used to fit desired path
//          Fit coeffs from waypoints
          // TODO: Safecasting using size_t?
          /**
           * Global to car
           */
          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          VectorXd ptsx_veh = VectorXd::Zero(ptsx.size());
          VectorXd ptsy_veh = VectorXd::Zero(ptsy.size());

          vector<double> ptxy;
          for (auto i = 0; i < ptsx.size() && i < ptsy.size(); i+=1) {
            ptxy = map2car(psi, ptsx[i], ptsy[i], px, py);

            ptsx_veh(i) = ptxy[0];
            next_x_vals.push_back(ptxy[0]);

            ptsy_veh(i) = ptxy[1];
            next_y_vals.push_back(ptxy[1]);
          }

          double x = 0;
          double y = 0;
          double psi_veh = 0;
          // Going to 3rd order
          VectorXd coeffs = polyfit(ptsx_veh, ptsy_veh, 3);
          v = mph_to_mps(v);
          double cte = get_cte(x, y, coeffs);
          double epsi = get_epsi(x, psi, coeffs);
          /*
           * To account for latency, predict the vehicle state 100ms into the future
           * before passing it to the solver. Then take the first actuator value
           */
          progress_state(&x, &y, &psi, &v, &cte, &epsi, steer_value, throttle_value, 0.1);

          // Augmented state vector
          // x, y, psi, cte, epsi in car coordinate
          // v in global coordinate
          VectorXd state = VectorXd::Zero(6);
          state << x, y, psi_veh, v, cte, epsi;

          // invoke solver
          vector<double> vars = mpc.Solve(state, coeffs);
          steer_value = mpc.steer_value;
          throttle_value = mpc.throttle_value;

          std::cout << "steer: " << steer_value << std::endl;
          std::cout << "throttle: " << throttle_value << std::endl;

          /**
           * Test only
           */
//          steer_value = 0;
//          throttle_value = 0;
          //Display the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          for (auto i=0; i < vars.size(); i+=2) {
            mpc_x_vals.push_back(vars[i]);
            mpc_y_vals.push_back(vars[i+1]);
          }

          json msgJson;

          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = -steer_value/deg2rad(25);
          //msgJson["steering_angle"] = -steer_value;
          msgJson["throttle"] = throttle_value;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
