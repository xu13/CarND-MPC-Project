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
constexpr double pi(){ return M_PI; }

double deg2rad(double x){ return x * pi() / 180; }

double rad2deg(double x){ return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s)
{
  auto found_null = s.find("null");
  auto b1         = s.find_first_of("[");
  auto b2         = s.rfind("}]");
  if (found_null != std::string::npos) {
    return "";
  } else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x)
{
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order)
{
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

  auto Q      = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

/*
 * Transform the list of waypoints in map frame to car frame.
 *
 */
void transform(const std::vector<double> ptsx, const std::vector<double> ptsy,
               double px, double py, double psi,
               Eigen::VectorXd& ptsx_car, Eigen::VectorXd& ptsy_car)
{
  assert(ptsx.size() == ptsy.size());
  size_t s = ptsx.size();
  ptsx_car = Eigen::VectorXd(s);
  ptsy_car = Eigen::VectorXd(s);
  for (size_t i = 0; i < ptsx.size(); i++) {
    ptsx_car[i] = std::cos(psi) * (ptsx[i] - px) + std::sin(psi) * (ptsy[i] - py);
    ptsy_car[i] = -std::sin(psi) * (ptsx[i] - px) + std::cos(psi) * (ptsy[i] - py);
  }
}

/*
 * Convert speed from mph to m/s.
 *
 */
double mph2mps(double v)
{
  return v * 0.44704;
}

Eigen::VectorXd simulate(double v, double steer, double throttle,
                         const Eigen::VectorXd& coeffs, double dt)
{
  // Calculate the cross track error
  double cte = polyeval(coeffs, 0) - 0;
  // printf("coeffs=[%f, %f, %f, %f]\n", coeffs[0], coeffs[1], coeffs[2], coeffs[3]);

  // Calculate the orientation error
  double epsi = 0 - std::atan(coeffs[1]);
  // printf("cte=%f, epsi=%f\n", cte, epsi);

  // Run a forward simulation for 100ms to account for latency
  static const double Lf = 2.67;
  double x_new    = v * dt;
  double y_new    = 0;
  double psi_new  = v / Lf * steer * dt;
  double v_new    = v + throttle * dt;
  double cte_new  = cte + v * std::sin(epsi) * dt;
  double epsi_new = epsi + v / Lf * steer * dt;

  Eigen::VectorXd state(6);
  state << x_new, y_new, psi_new, v_new, cte_new, epsi_new;

  return state;
}

int main()
{
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

  h.onMessage([&mpc, &t1](uWS::WebSocket<uWS::SERVER> ws, char* data, size_t length,
                          uWS::OpCode opCode){
    std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1);
    t1 = t2;
    std::cout << "**********" << std::endl;
    std::cout << "time: " << time_span.count() << std::endl;
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    std::string sdata = std::string(data).substr(0, length);
    // std::cout << sdata << std::endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      std::string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
 // j[1] is the data JSON object
          std::vector<double> ptsx = j[1]["ptsx"];
          std::vector<double> ptsy = j[1]["ptsy"];
          double px  = j[1]["x"];
          double py  = j[1]["y"];
          double psi = j[1]["psi"];
          double v   = j[1]["speed"];
          v = mph2mps(v);
          double steer_current    = j[1]["steering_angle"];
          double throttle_current = j[1]["throttle"];
          steer_current *= -1;
          printf("current steering: %f, current throttle: %f\n", steer_current, throttle_current);

 /*
  * TODO: Calculate steering angle and throttle using MPC.
  *
  * Both are in between [-1, 1].
  *
  */
 // Transform ptsx, ptsy from map frame to car frame
          Eigen::VectorXd ptsx_car;
          Eigen::VectorXd ptsy_car;
          transform(ptsx, ptsy, px, py, psi, ptsx_car, ptsy_car);

 // Fit the polynomial
          Eigen::VectorXd coeffs = polyfit(ptsx_car, ptsy_car, 3);

          // Forward simulation to account for latency
          double latency        = 0.1;
          Eigen::VectorXd state = simulate(v, steer_current, throttle_current, coeffs, latency);

 // Run MPC
          auto vars = mpc.Solve(state, coeffs);

          // Set the control command
          double steer_value    = vars[0];
          double throttle_value = vars[1];
          printf("command steering: %f, command throttle: %f\n", -steer_value, throttle_value);

          json msgJson;
 // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
 // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = -steer_value / deg2rad(25);
          msgJson["throttle"]       = throttle_value;

 // Display the MPC predicted trajectory
          std::vector<double> mpc_x_vals;
          std::vector<double> mpc_y_vals;

 // .. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
 // the points in the simulator are connected by a Green line
          for (size_t i = 2; i < vars.size(); i += 2) {
            mpc_x_vals.push_back(vars[i]);
            mpc_y_vals.push_back(vars[i + 1]);
          }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

 // Display the waypoints/reference line
          std::vector<double> next_x_vals;
          std::vector<double> next_y_vals;

 // .. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
 // the points in the simulator are connected by a Yellow line
          for (size_t i = 0; i < ptsx.size(); i++) {
            next_x_vals.push_back(ptsx_car[i]);
            next_y_vals.push_back(ptsy_car[i]);
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";

 // std::cout << msg << std::endl;
 // Latency
 // The purpose is to mimic real driving conditions where
 // the car does NOT actuate the commands instantly.
 //
 // Feel free to play around with this value but should be to drive
 // around the track with 100ms latency.
 //
 // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
 // SUBMITTING.
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
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
  h.onHttpRequest([](uWS::HttpResponse* res, uWS::HttpRequest req, char* data,
                     size_t, size_t){
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
 // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req){
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char* message, size_t length){
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
} // main
