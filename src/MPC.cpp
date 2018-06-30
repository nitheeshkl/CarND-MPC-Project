#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Set the timestep length and duration
size_t N = 10;
const double dt = 0.1;

const double Lf = 2.67;

const double ref_cte = 0;
const double ref_epsi = 0;
const double ref_v = 100;

// each variable is stored in sequence for a set of N values.
// hence we add corresponding offset to index them later
const size_t x_index = 0;
const size_t y_index = x_index + N;
const size_t psi_index = y_index + N;
const size_t v_index = psi_index + N;
const size_t cte_index = v_index + N;
const size_t epsi_index = cte_index + N;
const size_t delta_index = epsi_index + N;
const size_t acc_index = delta_index + N - 1;


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

    // initialize cost
    fg[0] = 0;

    // add cost from ref state
    for (unsigned int i = 0; i < N; i++) {
      fg[0] += 1000*CppAD::pow(vars[cte_index + i] - ref_cte, 2);
      fg[0] += 1000*CppAD::pow(vars[epsi_index + i] - ref_epsi, 2);
      fg[0] += CppAD::pow(vars[v_index + i] - ref_v, 2);

      if (i < N-1) {
        fg[0] += 50*CppAD::pow(vars[delta_index + i], 2);
        fg[0] += 50*CppAD::pow(vars[acc_index + i], 2);
      }

      if (i < N-2) {
        fg[0] += 250000*CppAD::pow(vars[delta_index + i + 1] - vars[delta_index + i], 2);
        fg[0] += 5000*CppAD::pow(vars[acc_index + i + 1] - vars[acc_index + i], 2);
      }
    }

    // initialize constrains
    fg[1 + x_index] = vars[x_index];
    fg[1 + y_index] = vars[y_index];
    fg[1 + psi_index] = vars[psi_index];
    fg[1 + v_index] = vars[v_index];
    fg[1 + cte_index] = vars[cte_index];
    fg[1 + epsi_index] = vars[epsi_index];

    for (unsigned int t = 1; t < N; t++) {
      // state at t
      AD<double> x_t = vars[x_index + t];
      AD<double> y_t = vars[y_index + t];
      AD<double> psi_t = vars[psi_index + t];
      AD<double> v_t = vars[v_index + t];
      AD<double> cte_t = vars[cte_index + t];
      AD<double> epsi_t = vars[epsi_index + t];

      // previous state: at t-1
      AD<double> x_prev_t = vars[x_index + t-1];
      AD<double> y_prev_t = vars[y_index + t-1];
      AD<double> psi_prev_t = vars[psi_index + t-1];
      AD<double> v_prev_t = vars[v_index + t-1];
      AD<double> cte_prev_t = vars[cte_index + t-1];
      AD<double> epsi_prev_t = vars[epsi_index + t-1];

      AD<double> delta_prev_t = vars[delta_index + t-1];
      AD<double> acc_prev_t = vars[acc_index + t-1];

      AD<double> f_prev_t = coeffs[0]
                            + coeffs[1]*x_prev_t
                            + coeffs[2]*CppAD::pow(x_prev_t, 2)
                            + coeffs[3]*CppAD::pow(x_prev_t, 3);

      AD<double> psid_prev_t = CppAD::atan(coeffs[1]
                                            + 2*coeffs[2]*x_prev_t
                                            + 3*coeffs[3]*CppAD::pow(x_prev_t, 2)
                                          );

      fg[1 + x_index + t] = x_t - (x_prev_t + v_prev_t*CppAD::cos(psi_prev_t)*dt);
      fg[1 + y_index + t] = y_t - (y_prev_t + v_prev_t*CppAD::sin(psi_prev_t)*dt);
      fg[1 + v_index + t] = v_t - (v_prev_t + acc_prev_t*dt);
      fg[1 + psi_index + t] = psi_t - (psi_prev_t - v_prev_t/Lf * delta_prev_t * dt);
      fg[1 + cte_index + t] = cte_t - ((f_prev_t - y_prev_t) + (v_prev_t*CppAD::sin(epsi_prev_t)*dt));
      fg[1 + epsi_index + t] = epsi_t - ((psi_prev_t - psid_prev_t) - v_prev_t/Lf*delta_prev_t*dt);
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

  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  size_t n_vars = N*6 + (N-1)*2;
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

  for (unsigned int i = 0; i < delta_index; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  for (unsigned int i = delta_index; i < acc_index; i++) {
    vars_lowerbound[i] = -0.436332*Lf;
    vars_upperbound[i] = 0.436332*Lf;
  }

  for (unsigned int i = acc_index; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[x_index] = state[0];
  constraints_lowerbound[y_index] = state[1];
  constraints_lowerbound[psi_index] = state[2];
  constraints_lowerbound[v_index] = state[3];
  constraints_lowerbound[cte_index] = state[4];
  constraints_lowerbound[epsi_index] = state[5];

  constraints_upperbound[x_index] = state[0];
  constraints_upperbound[y_index] = state[1];
  constraints_upperbound[psi_index] = state[2];
  constraints_upperbound[v_index] = state[3];
  constraints_upperbound[cte_index] = state[4];
  constraints_upperbound[epsi_index] = state[5];

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

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  vector<double> result;

  result.push_back(solution.x[delta_index]);
  result.push_back(solution.x[acc_index]);

  for (unsigned int i = 0; i < N-2; i++) {
    result.push_back(solution.x[x_index + i+1]);
    result.push_back(solution.x[y_index + i+1]);
  }

  return result;
}
