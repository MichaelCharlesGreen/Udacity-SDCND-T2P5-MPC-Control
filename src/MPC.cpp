#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// L19M5
// The prediction horizon is the duration over which future predictions are made.
// The prediction horizon is the product of N and dt (prediction_horizon = N*dt).
// The prediction horizon should be as large as possible - a few seconds, at most.
// dt should be as small as possible.
// The goal of MPC is to optimize the control inputs: [delta,a].
// An optimizer will tune these inputs until a low cost vector of control inputs is found.
// The length of this vector is determined by N.
// Thus, N determines the number of variables optimized by the MPC.
// This is also the major driver of computational cost.
// MPC attempts to approximate a continuous reference trajectory by means of discrete patha
// begtween actuations. Larger values of dt result in less frequent actuations, which makes
// it harder to accurately approximate a continuous reference trajectory. This is sometimes
// called "discretization error".
// L19M6
// Model predictive control uses an optimizer to find the control inputs and minimize the cost function.
// We actually only execute the very first set of control inputs.
// This brings the vehile to a new state and then you repeat the process.
// The model, cost and constraints comprise the solver.
// The solver we'll use is called Ipopt.
//
// A good approach to setting N and dt is to first determing a reasonable range for the
// prediction horizon and then tune dt and N appropriately, keeping the effect of each in mind.

// TODO: Set the timestep length and duration
size_t N = 10; // the number of timesteps in the horizon
double dt = 0.1; // how much time elapses between actuations

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
const double Lf = 2.67;

double ref_v = 75;
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

class FG_eval {
public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }
  
  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
	// TODO: implement MPC
	// `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
	// NOTE: You'll probably go back and forth between this function and
	// the Solver function below.
	
	// L19M9
	// The vector fg is where teh cost function and vehicle model/constraints are defined.
	// Since 0 is the index at which Ipopt expects fg to store the cost value, we sum all the components
	// of the cost and store them at index 0.
	fg[0] = 0;
	
	// L19M2
	// A good start to the cost function is to think of the error that you would like to minimize.
	// For example, measuring the offset from the center of the lane, where the center of the lane
	// can be called the reference, or desired, state.
	// Ideally, there would be no differene from the actual vehicle position and heading to the
	// desired position and heading.
	
	// Increment the cost (fg[0]) at each timestep.
	
	// L19M9
	// Cost function
	// L19M10
	// Tuning MPC
	
	// In each iteration through the loop, we sum three components to reach the aggregate cost:
	// our cross-track error, our heading error, and our velocity error.
	
	// Reference State Cost
	// TODO: Define the cost related the reference state and
	// any anything you think may be beneficial.
	
	// The part of the cost based on the reference state.
	for (int i = 0; i < N; i++) {
	  fg[0] += 1000*CppAD::pow(vars[cte_start + i], 2); // cross track error
	  fg[0] += 1000*CppAD::pow(vars[epsi_start + i], 2); // orientation error
	  fg[0] += 1*CppAD::pow(vars[v_start + i] - ref_v, 2); // velocity error
	}
	
	// Minimize the use of actuators.
	// Minimize change-rate.
	// Constrain erratic control inputs.
	// For example, if we're making a turn, we'd like the turn to be smooth, not sharp.
	// Additionally, the vehicle velocity should not change too radically.
	for (int i = 0; i < N - 1; i++) {
	  fg[0] += 1*CppAD::pow(vars[delta_start + i], 2);
	  fg[0] += 1*CppAD::pow(vars[a_start + i], 2);
	  // a penalty for velocity and steering
	  fg[0] += 100*CppAD::pow(vars[delta_start + i] * vars[v_start+i], 2);
	}
	
	// Minimize the value gap between sequential actuations.
	// Make control decisions more consistent, or smoother.
	// The next control input should be similar to the current one.
	for (int i = 0; i < N - 2; i++) {
	  // Multiplying by a value > 1 will influence the solver into keeping sequential steering values closer together.
	  fg[0] += 20*CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
	  fg[0] += 10*CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
	}
	
	//
	// Setup Constraints
	//
	// NOTE: In this section you'll setup the model constraints.
	
	// Initial constraints
	//
	// We add 1 to each of the starting indices due to cost being located at
	// index 0 of `fg`.
	// This bumps up the position of all the other values.
	fg[1 + x_start] = vars[x_start];
	fg[1 + y_start] = vars[y_start];
	fg[1 + psi_start] = vars[psi_start];
	fg[1 + v_start] = vars[v_start];
	fg[1 + cte_start] = vars[cte_start];
	fg[1 + epsi_start] = vars[epsi_start];
	
	// L19M8
	// In order to used CppAD effectively, we have to use its types instead of regular double or std::vector types.
	// Additionally, math functions must be called from CppAD.
	// Luckily, most elementary math operations are overloaded. So calling *, +, -, / will work as intended as long
	// as it's called on CppAD<double> instead of double.
	
	// The rest of the constraints
	for (int t = 1; t < N; t++) {
	  // motion [x,y,psi,v,cte,epsi]
	  // next
	  AD<double> x1 = vars[x_start + t];
	  AD<double> y1 = vars[y_start + t];
	  AD<double> psi1 = vars[psi_start + t];
	  AD<double> v1 = vars[v_start + t];
	  AD<double> cte1 = vars[cte_start + t];
	  AD<double> epsi1 = vars[epsi_start + t];
	  // current
	  AD<double> x0 = vars[x_start + t - 1];
	  AD<double> y0 = vars[y_start + t - 1];
	  AD<double> psi0 = vars[psi_start + t - 1];
	  AD<double> v0 = vars[v_start + t - 1];
	  AD<double> cte0 = vars[cte_start + t - 1];
	  AD<double> epsi0 = vars[epsi_start + t - 1];
	  // control [delta,a]
	  // L19M9
	  // Only consider the actuation at time t.
	  AD<double> delta = vars[delta_start + t - 1];
	  AD<double> a = vars[a_start + t - 1];
	  
	  //AD<double> f0 = coeffs[0] + coeffs[1] * x0;
	  //AD<double> psides0 = CppAD::atan(coeffs[1]);
	  
	  // Account for latency by using the previous actuations.
	  if (t > 1) {
		a = vars[a_start + t - 2];
		delta = vars[delta_start + t - 2];
	  }
	  
	  AD<double> f0 = coeffs[0] + coeffs[1]*x0 + coeffs[2]*CppAD::pow(x0, 2) + coeffs[3]*CppAD::pow(x0, 3);
	  AD<double> psides0 = CppAD::atan(coeffs[1] + 2*coeffs[2]*x0 + 3*coeffs[3]*CppAD::pow(x0, 2));
	  
	  // Here's `x` to get you started.
	  // The idea here is to constraint this value to be 0.
	  //
	  // Recall the equations for the model:
	  // x_[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
	  // y_[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
	  // psi_[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
	  // v_[t] = v[t-1] + a[t-1] * dt
	  // cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
	  // epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt
	  fg[1 + x_start + t] = x1 - (x0 + v0*CppAD::cos(psi0)*dt);
	  fg[1 + y_start + t] = y1 - (y0 + v0*CppAD::sin(psi0)*dt);
	  fg[1 + psi_start + t] = psi1 - (psi0 - v0/Lf*delta*dt);
	  fg[1 + v_start + t] = v1 - (v0 + a*dt);
	  fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0*CppAD::sin(epsi0)*dt));
	  fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) - v0/Lf*delta*dt);
	}
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;
  
  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];
  
  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  size_t n_vars = N*6 + (N - 1)*2;
  // TODO: Set the number of constraints
  size_t n_constraints = N*6;
  
  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
	vars[i] = 0;
  }
  
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.
  
  

  // L19M8
  // Set the initial variable values
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;
  
  // Set lower and upper bounds on the variables.
  for (int i = 0; i < delta_start; i++) {
	vars_lowerbound[i] = -1.0e19;
	vars_upperbound[i] = 1.0e19;
  }
  
  // delta (steering angle) set to -25 and 25 degrees (values in radians).
  for (int i = delta_start; i < a_start; i++) {
	vars_lowerbound[i] = -0.436332;
	vars_upperbound[i] = 0.436332;
  }
  
  // Acceleration upper and lower limits.
  for (int i = a_start; i < n_vars; i++) {
	vars_lowerbound[i] = -1.0;
	vars_upperbound[i] = 1.0;
  }
  
  // Constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
	constraints_lowerbound[i] = 0;
	constraints_upperbound[i] = 0;
  }
  
  // lowerbound constraints
  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;
  // upperbound constraints
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
  options += "Integer print_level  0\n";
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
  
  // L19M8
  // Ipopt is the tool to optimize the control inputs. It's able to find locally optimal values (non-liner problem!)
  // while keeping the contraints set directly to the actuators and the contraints defined by the vehicle model.
  // Ipopt requires we give it the jacobians and hessians directly - it does not compute them for us. Hence, we need
  // to either manually compute them or have a library do this for us.
  // Luckily, there is a library called CppAD which does exactly this.
  // CppAD is a library we'll use for automatice differentiation. By using CppAD we don't have to manually compute
  // derivatives, which is tedious and prone to error.
  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;
  
  std::cout << "1" << std::endl;
  std::cout << options << std::endl;
  std::cout << vars << std::endl;
  std::cout << vars_lowerbound << std::endl;
  std::cout << vars_upperbound << std::endl;
  std::cout << constraints_lowerbound << std::endl;
  std::cout << constraints_upperbound << std::endl;
  //  std::cout << fg_eval << std::endl;
  //  std::cout << solution << std::endl;
  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
										options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
										constraints_upperbound, fg_eval, solution);
  std::cout << "2" << std::endl;
  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
  std::cout << "3" << std::endl;
  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;
  std::cout << "4" << std::endl;
  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  vector<double> result;
  
  result.push_back(solution.x[delta_start]);
  result.push_back(solution.x[a_start]);
  
  for (int i = 0; i < N-1; i++) {
	result.push_back(solution.x[x_start + i + 1]);
	result.push_back(solution.x[y_start + i + 1]);
  }
  
  return result;
}
