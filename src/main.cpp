#include <math.h>
#include <uWS/uWS.h>
#include <ctime>
#include <ratio>
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
  auto result = Q.solve(yvals);
  return result;
}

double ref_v;
double dt;

int main(int argc, char** argv) {
  double dt_run = atof(argv[3]);
  ref_v = atof(argv[2]);
  dt = atof(argv[1]);
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc, &dt_run](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
          vector<double> ptsx_ = j[1]["ptsx"];
          vector<double> ptsy_ = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double psi_unity = j[1]["psi_unity"];
          double v = j[1]["speed"];
          double steering_angle = j[1]["steering_angle"];

          /*
          * Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          vector<double> ptsx_tr, ptsy_tr;
          for(size_t i = 0; i < ptsx_.size(); i++) {
            double x =  (ptsx_[i] - px)*cos(- psi) - (ptsy_[i] - py)*sin(- psi);
            double y =  (ptsx_[i] - px)*sin(- psi) + (ptsy_[i] - py)*cos(- psi);
            ptsx_tr.push_back(x);
            ptsy_tr.push_back(y);
          }
          Eigen::VectorXd ptsx(ptsx_.size());
          Eigen::VectorXd ptsy(ptsy_.size());
          ptsx << ptsx_tr[0], ptsx_tr[1], ptsx_tr[2], ptsx_tr[3], ptsx_tr[4], ptsx_tr[5];
          ptsy << ptsy_tr[0], ptsy_tr[1], ptsy_tr[2], ptsy_tr[3], ptsy_tr[4], ptsy_tr[5];
          auto coeffs = polyfit(ptsx, ptsy, 2);

          // NOTE: free feel to play around with these
          double x = 0.0;
          double y = 0.0;
          // The cross track error is calculated by evaluating at polynomial at x, f(x)
          // and subtracting y.
          double cte = polyeval(coeffs, x) - y;
          std::cout << "cte is " << cte << std::endl;
          // Due to the sign starting at 0, the orientation error is -f'(x).
          // derivative of coeffs[0] + coeffs[1] * x -> coeffs[1]
          double epsi = -atan(coeffs[1]);
          std::cout << "epsi is " << epsi << std::endl;

          Eigen::VectorXd state(6);
          state << x, y, 0, v, cte, epsi;

          std::vector<double> x_vals = {state[0]};
          std::vector<double> y_vals = {state[1]};
          std::vector<double> psi_vals = {state[2]};
          std::vector<double> v_vals = {state[3]};
          std::vector<double> cte_vals = {state[4]};
          std::vector<double> epsi_vals = {state[5]};
          std::vector<double> delta_vals = {};
          std::vector<double> a_vals = {};
          
          //#size_t iters = 10;
          //#for (size_t i = 0; i < iters; i++) {
            //std::cout << "Iteration " << i << std::endl;

            auto vars = mpc.Solve(state, coeffs);
            
            const size_t N = 24;
            for(size_t i = 2; i < N; i+=3) {
              double x = vars[2*i];
              double y = vars[2*i+1];
              x_vals.push_back(x);
              y_vals.push_back(y);
            }

            psi_vals.push_back(vars[2*N]);
            v_vals.push_back(vars[2*N+1]);
            cte_vals.push_back(vars[2*N+2]);
            epsi_vals.push_back(vars[2*N+3]);

            delta_vals.push_back(vars[2*N+4]);
            a_vals.push_back(vars[2*N+5]);

            state << vars[0], vars[1], vars[2*N], vars[2*N+1], vars[2*N+2], vars[2*N+3];
            std::cout << "x = " << vars[0] << std::endl;
            std::cout << "y = " << vars[1] << std::endl;
            std::cout << "psi = " << vars[2*N] << std::endl;
            std::cout << "v = " << vars[2*N+1] << std::endl;
            std::cout << "cte = " << vars[2*N+2] << std::endl;
            std::cout << "epsi = " << vars[2*N+3] << std::endl;
            std::cout << "delta = " << vars[2*N+4] << std::endl;
            std::cout << "a = " << vars[2*N+5] << std::endl;
            std::cout << std::endl;
          //}
          static chrono::high_resolution_clock::time_point t1 = chrono::high_resolution_clock::now();
          chrono::high_resolution_clock::time_point t2 = chrono::high_resolution_clock::now();

          chrono::duration<double> time_span = chrono::duration_cast<chrono::duration<double>>(t2 - t1);

          double dt = time_span.count();
          std::cout << "dt = " << dt << " seconds.";
          t1 = t2;
          const double Lf = 2.67;
          double steer_value = - (v*dt_run*delta_vals[0]/Lf)/deg2rad(25);
          std::cout << "Steering Angle is " << steer_value << std::endl;
          double throttle_value = a_vals[0];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals = x_vals;
          vector<double> mpc_y_vals = y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals = ptsx_tr;
          vector<double> next_y_vals = ptsy_tr;

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
