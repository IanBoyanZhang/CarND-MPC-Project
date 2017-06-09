#include <math.h>
#include <uWS/uWS.h>
#include <thread>
#include "MPC.h"
#include "Tools.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;
using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

Tools tools;

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

int main(int argc, const char *argv[]) {
  uWS::Hub h;

  vector<double> hyper_params;

  /*************************************************************************
   * Cost weights hyper parameters
   *************************************************************************/
  /*w_cost_ref_cte = hyper_params[0];
  w_cost_ref_epsi = hyper_params[1];
  w_cost_ref_v = hyper_params[2];
  w_cost_ref_val_steering = hyper_params[3];
  w_cost_ref_val_throttle = hyper_params[4];
  w_cost_ref_seq_steering = hyper_params[5];
  w_cost_ref_seq_throttle = hyper_params[6];*/
  if (argc != 8) {
    cout << "Please see ./run.sh for example, now running with default parameters" << endl;
    hyper_params.push_back(1);
    hyper_params.push_back(500);
    hyper_params.push_back(1);
    hyper_params.push_back(300);
    hyper_params.push_back(1);
    hyper_params.push_back(300);
    hyper_params.push_back(1);
  } else {
    for (auto i = 1; i < 8; i+=1) {
      double _val = strtod(argv[i], NULL);
      hyper_params.push_back( _val );
      std::cout << _val << std::endl;
    }
  }

  // MPC is initialized here!
  MPC mpc(hyper_params);

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
           * Both are in between [-1, 1].
           */
          double steer_value = j[1]["steering_angle"];
          double throttle_value = j[1]["throttle"];

          /*************************************************************************
           * Transform way points coordinates to vehicle coordinates
           *************************************************************************/
          /*************************************************************************
           * Server usually provides 6 navigation points
           * Used for fitting desired trajectory
           *************************************************************************/
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          VectorXd ptsx_veh = VectorXd::Zero(ptsx.size());
          VectorXd ptsy_veh = VectorXd::Zero(ptsy.size());

          vector<double> ptxy;
          for (auto i = 0; i < ptsx.size() && i < ptsy.size(); i+=1) {
            ptxy = tools.map2car(psi, ptsx[i], ptsy[i], px, py);

            ptsx_veh(i) = ptxy[0];
            next_x_vals.push_back(ptxy[0]);

            ptsy_veh(i) = ptxy[1];
            next_y_vals.push_back(ptxy[1]);
          }

          double x = 0;
          double y = 0;
          double psi_veh = 0;
          /*************************************************************************
           * 3rd order trajectory fitting using navigation points
           *************************************************************************/
          VectorXd coeffs = tools.polyfit(ptsx_veh, ptsy_veh, 3);
          v = tools.mph_to_mps(v);
          double cte = tools.get_cte(x, y, coeffs);
          double epsi = tools.get_epsi(x, psi, coeffs);
          /*************************************************************************
           * To account for latency, predict the vehicle state 100ms into the future
           * before passing it to the solver. Then take the first actuator value
           * x, y, psi, cte, epsi Vehicle coordinate
           * v Global coordinate
           *************************************************************************/
          tools.progress_state(&x, &y, &psi, &v, &cte, &epsi, steer_value, throttle_value, latency, Lf);

          VectorXd state = VectorXd::Zero(6);
          state << x, y, psi_veh, v, cte, epsi;
          /*************************************************************************
           * Internal points solver for cost optimization (minimization)
           *************************************************************************/
          vector<double> vars = mpc.Solve(state, coeffs);
          steer_value = mpc.steer_value;
          throttle_value = mpc.throttle_value;

          std::cout << "steer: " << steer_value << std::endl;
          std::cout << "throttle: " << throttle_value << std::endl;

          //Display the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;
          /*************************************************************************
           * Unpack returned predicted trajectory from Optimization Solver
           *************************************************************************/
          for (auto i=0; i < vars.size(); i+=2) {
            mpc_x_vals.push_back(vars[i]);
            mpc_y_vals.push_back(vars[i+1]);
          }

          /*************************************************************************
           * Sending commands back to server
           *************************************************************************/
          json msgJson;

          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          //msgJson["steering_angle"] = -steer_value/deg2rad(25);
          msgJson["steering_angle"] = -steer_value;
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
