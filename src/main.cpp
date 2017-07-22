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
	//cout << sdata << endl;
	if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
	  string s = hasData(sdata);
	  if (s != "") {
		auto j = json::parse(s);
		string event = j[0].get<string>();
		if (event == "telemetry") {
		  // j[1] is the data JSON object
		  vector<double> ptsx = j[1]["ptsx"]; // waypoints-x
		  vector<double> ptsy = j[1]["ptsy"]; // waypoints-y
		  double px = j[1]["x"]; // car x-position
		  double py = j[1]["y"]; // car y-position
		  double psi = j[1]["psi"]; // car psi
		  double v = j[1]["speed"]; // car velocity
		  
		  // Transform pts (waypoints) to be the car's coordinate system.
		  for (int i = 0; i < ptsx.size(); i++ )
		  {
			// Shift car reference angle to 90 degrees.
			double dx = ptsx[i]-px; // delta x (waypoint to car)
			double dy = ptsy[i]-py; // delta y (waypoint to car)
			ptsx[i] = (dx*cos(0-psi) - dy*sin(0-psi));
			ptsy[i] = (dx*sin(0-psi) + dy*cos(0-psi));
		  }
		  // Pointers to waypoints.
		  double * ptrx = &ptsx[0];
		  double * ptry = &ptsy[0];
		  // Using six waypoints.
		  Eigen::Map<Eigen::VectorXd> ptsx_transform(ptrx, 6);
		  Eigen::Map<Eigen::VectorXd> ptsy_transform(ptry, 6);
		  // Fit a 3rd degree polynomial to the transformed waypoints.
		  // coeffs are the coefficients of the polynomial.
		  // L18M7
		  // polyfit fits a 3rd order polynomial to the given x and y
		  // coordinates representing waypoints.
		  // L19M8
		  // coeffs are the coefficients of the fitting polynomial
		  auto coeffs = polyfit(ptsx_transform, ptsy_transform, 3);
		  // cross-track error
		  // The measure of the distance of the car from the polynomial
		  // guide line created from the waypoints.
		  // L18M7
		  // polyeval evaluates y values of given x coordinates
		  // L18M8
		  // The distance of the vehicle from trajectory and difference
		  // of vehicle orientation and trajectory orientation are the
		  // errors to minimize.
		  // We can capture how the errors are interested in change over
		  // time by deriving the kinematic model around these errors as
		  // the new state vector. [x,y,psi,v,cte,epsi]
		  double cte = polyeval(coeffs, 0);
		  // Error in the car's orientation measured against the tangent
		  // of the guide line created from the waypoints.
		  double epsi = -atan(coeffs[1]);
		  
		  double steer_value = j[1]["steering_angle"];
		  double throttle_value = j[1]["throttle"];
		  
		  // L19M7
		  // Latency
		  // In a real car, an actuation command won't execute instantly - there
		  // will be a delay as the command propagates through the system. A
		  // realistic delay might be on the order of 100 milliseconds.
		  // This is a problem called "latency". While it is a problem for a PID
		  // controller to overcome, a Model Predictive Controll can adapt quite
		  // well because we can model this latency in the system.
		  // A contributing factor to latency is actuator dynamics. For example,
		  // the elapsed time between when a command is issued and the result is
		  // achieved.
		  // This could easily be modeled by a simple dynamic system and incorporated
		  // into the vehicle model. One approach would be running a simulation using
		  // the vehicle model starting from the current state for the duration of the
		  // latency. The resulting state from the simulation is the new initial state
		  // for MPC.
		  double delay_t = .1; // represents the delay of actuators in seconds
		  // L18M4
		  // Lf measures the distance between the front of the vehicle and its center
		  // of gravity. The larger the vehicle, the slower the turn rate.
		  // Turns are quicker at higher speeds than at lower speeds; this is why v is
		  // included in the update: psi = psi + v/Lf*delta*dt where delta is the
		  // steering angle.
		  // The validity of a vehicle's motion model is tested by running a vehicle
		  // in a circle. If the radius of the circle generated from driving the test
		  // vehicle around in a circle with a constant velocity and steering angle is
		  // similar to that of the model in the simulation, then this is a validation.
		  const double Lf = 2.67; // calculated in simulator by driving in a circle
		  
		  // L18M2
		  // Kinematic models are simplifications of dynamic models that ignore tire
		  // forces, gravity, and mass. This simplification reduces the accuracy of
		  // the models, but it also makes them more tractable. At low and moderate
		  // speeds, kinematic models often approximate the actual vehicle dynamics.
		  //
		  // Dynamic models aim to embody the actual vehicle dynamics as closely as
		  // possible. They might encompass tire forces, longitudinal and lateral
		  // forces, inertia, gravity, air resistance, drag, mass and the geometry of
		  // the vehicle.
		  
		  // TODOmg: implement latency
		  // Accounting for the delay in the system.
		  // L18M5
		  // Global kinematic model for motion of the vehicle.
		  // xt+1 = xt + vt*cos(psit)*dt
		  // L19M7
		  // Latency and MPC
		  // Running a simulation using the vehicle model starting
		  // from the current state for the duration of the latency
		  // models latency. The resulting state from this simulation
		  // is the new initial state for MPC.
//		  double delay_x = v*delay_t;
//		  double delay_y = 0;
//		  double delay_psi = -v*steer_value/Lf*delay_t;
//		  double delay_v = v + throttle_value*delay_t;
//		  double delay_cte = cte + v*sin(epsi)*delay_t;
//		  double delay_epsi = epsi-v*steer_value/Lf*delay_t;
		  
		  Eigen::VectorXd state(6);
//		  state << delay_x, delay_y, delay_psi, delay_v, delay_cte, delay_epsi;
		  state << 0, 0, 0, v, cte, epsi;
		  
		  /*
		   * TODO: Calculate steeering angle and throttle using MPC.
		   *
		   * Both are in between [-1, 1].
		   *
		   */
		  
		  // L19M8
		  // Ipopt is the tool we'll be using to optimize the control inputs.
		  // Variables for Ipopt.
		  // Ipopt expects all the constraints
		  // and variables as vectors.
		  // The number of elements of the vector
		  // depends on the value of N.
		  // If N is 5, then the structure of vars
		  // is a 38-element vector (6*5 + 2*4)
		  // where the first 30 elements correspond
		  // to x,y,psi,v,cte,epsi and the next 8
		  // elements correspond to the constraints
		  // delta and a.
		  // delta (the steering angle) is typically
		  // constraint to +- 25 degrees in radians
		  // a (acceleration) is typically constrainted
		  // between +-1 (representing braking and
		  // speeding up).
		  // L19M9
		  // The vars vector contains all variables used by the cost function and model.
		  // [x,y,psi,v,cte,epsi] and [delta,a]
		  auto vars = mpc.Solve(state, coeffs);
		  steer_value = vars[0];
		  throttle_value = vars[1];
		  
		  // Yellow line in Simulator; the line to follow.
		  // Line formed by polyfitting the waypoints.
		  // L19M5
		  // The prediction horizon is the duration over which future predictions are made.
		  // The prediction horizon is the product of N and dt (10 and 0.1)
		  vector<double> next_x_vals;
		  vector<double> next_y_vals;
		  // N = 10; dt = 0.1
		  double poly_inc = 2.5; // x-value increment
		  int num_points = 25;
		  for (int i = 1; i < num_points; i++)
		  {
			next_x_vals.push_back(poly_inc*i);
			next_y_vals.push_back(polyeval(coeffs, poly_inc*i));
		  }
		  
		  // MPC predicted trajectory.
		  // Green line in Simulator; vehicle's predicted path.
		  vector<double> mpc_x_vals;
		  vector<double> mpc_y_vals;
		  for (int i = 2; i < vars.size(); i++)
		  {
			if(i%2 == 0)
			{
			  mpc_x_vals.push_back(vars[i]);
			}
			else
			{
			  mpc_y_vals.push_back(vars[i]);
			}
		  }
		  
		  json msgJson;
		  // [-1,1] range; 25 is the constraint on the steering angle
		  msgJson["steering_angle"] = steer_value/deg2rad(25);
		  msgJson["throttle"] = throttle_value;
		  
		  msgJson["next_x"] = next_x_vals;
		  msgJson["next_y"] = next_y_vals;
		  
		  msgJson["mpc_x"] = mpc_x_vals;
		  msgJson["mpc_y"] = mpc_y_vals;
		  
		  auto msg = "42[\"steer\"," + msgJson.dump() + "]";
		  //std::cout << msg << std::endl;
		  // Latency
		  // The purpose is to mimic real driving conditions where
		  // the car does actuate the commands instantly.
		  //
		  // Feel free to play around with this value but should be to drive
		  // around the track with 100ms latency.
		  //
		  // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
		  // SUBMITTING.
		  this_thread::sleep_for(chrono::milliseconds((int)(delay_t*1000)));
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
