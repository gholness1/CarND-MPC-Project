#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

using CppAD::AD;

/***
 * Set the timestamp length and duration
 *
 * Lots of experiementation done.  Started with
 * MPC Quiz values N=20, dt=0.05 and worked from
 * there. These values N=30 dt=0.03 seem to work well.
 *
 */
size_t N = 30;
double dt = 0.03;


/***
 * Set the reference velocity to 100, picked arbitrarily to see
 * how fast I could get the car to go.
 *
 * Both the reference cross track and orientation errors are 0.
 * The reference velocity is set to 40 mph.
 */
double ref_v = 100;

/***
 * Offsets within the state array for various parts of state
 */
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
  const double Lf= 2.67;
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.

    /****
     * Compute the cost function and store in position 0
     *
     * Included a scalar multiplier to emphasize different aspects
     * contributing to the cost function.  Highest emphasis is on
     * smooth actuation, that is, the actuation's time-based derivative
     * should be small.  This helps with runaway oscillation and encourages
     * the solver to select smooth driving actions (steering, throttle).
     */
    fg[0] = 0;

    /***
     *
     * The part of the cost based on the reference state.
     * Cross Track Error (CTE) is important since it measures
     * how "off center" the car is on the track.
     */
    for (int t = 0; t < N; t++) {
      fg[0] += 100* CppAD::pow(vars[cte_start + t], 2);
      fg[0] += CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
    }

    /***
     * Minimize the use of actuators.
     *
     * Actuation is more important than cross track because we
     * don't want huge actuation values (like stomping on accelerator
     * or jerking the steering wheel very hard).
     */
    for (int t = 0; t < N - 1; t++) {
      fg[0] += 500* CppAD::pow(vars[delta_start + t], 2);
      fg[0] += 500* CppAD::pow(vars[a_start + t], 2);
    }

    /***
     *
     * Minimize the value gap between sequential actuations, i.e. rate of change in actuation
     *
     * We want smooth driving actions so we have the highest modulation on the derivative
     * (one-step temporal difference) of the actuation signal.  This encourages solver to
     * select solutions that have smooth driving actuations.
     */
    for (int t = 0; t < N - 2; t++) {
      fg[0] += 2000* CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += 2000* CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }


    /****
     * Setup the constraints
     *
     * The indeces have the +1 because the cost function is at position
     * 0 and we need to account for that by making sure each constraint
     * occupies positions that are later in the array.
     *
     */

    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // The rest of the constraints
    for (int t = 1; t < N; t++) {
      // The state at time t+1 .
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];

      // The state at time t.
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];

      // Only consider the actuation at time t.
      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0 = vars[a_start + t - 1];

      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] *CppAD::pow(x0,2) + coeffs[3] *CppAD::pow(x0,3);
      AD<double> psides0 = CppAD::atan(coeffs[1]);

      // Here's `x` to get you started.
      // The idea here is to constraint this value to be 0.
      //
      // Recall the equations for the model:
      // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      // psi_[t+1] = psi[t] + v[t] / this->Lf * delta[t] * dt
      // v_[t+1] = v[t] + a[t] * dt
      // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt

      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);

      /***
       * Remember the negative delta mentioned in project pointers
       */
      fg[1 + psi_start + t] = psi1 - (psi0 - v0 * delta0 / Lf * dt);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t] =
          cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));

      /***
       * Remember the negative delata mentioned in project pointers
       */
      fg[1 + epsi_start + t] =
          epsi1 - ((psi0 - psides0) - v0 * delta0 / Lf * dt);
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

  /****
   *
   * TODO: Set the number of model variables (includes both states and inputs).
   * For example: If the state is a 4 element vector, the actuators is a 2
   * element vector and there are 10 timesteps. The number of variables is:
   * 
   * 4 * 10 + 2 * 9
   *
   * These were set identitcal to the MPC to line Quiz
   */

  size_t n_vars = N * 6 + (N - 1) * 2;
  // TODO: Set the number of constraints

  /****
   * N timesteps = N - 1 actuation steps
   */
  size_t n_constraints = N * 6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

#if 0
  /****
   * Note:  Did not need to do this, eliminated from code carried from 
   *        MPC Quiz
   *
   *            
   *Set the initial variable values
   */
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;
#endif


  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.

  /****
   * Set all non-actuators upper and lowerlimits
   * to the max negative and positive values.
   */
  for (int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

 /*** 
  * The upper and lower limits of delta are set to -25 and 25
  * degrees (values in radians). Did not change.
  *
  *  NOTE: Feel free to change this to something else.
  */
  for (int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

 /***
  * Acceleration/decceleration upper and lower limits. Did not change
  *
  * NOTE: Feel free to change this to something else.
  */
  for (int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }


  /***
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

 /***
  * Instantiate calculation of the objective
  *
  * object that computes objective and constraints
  */
  FG_eval fg_eval(coeffs);

 /***
  * Not messing with this stuff from the solver.
  * It works and that's good enough for me.
  */

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


  vector<double> output_solution;

  output_solution.push_back(solution.x[delta_start]);
  output_solution.push_back(solution.x[a_start]);

  for (int i= 0; i < N; i++) {
    output_solution.push_back(solution.x[x_start + i]);
    output_solution.push_back(solution.x[y_start + i]);
  }

  return output_solution;

#if 0
  /***
   * Previous incarnation of code was drastically wrong.
   * Removed it from code and re-implemented above
   */
  return {solution.x[x_start + 1],   solution.x[y_start + 1],
          solution.x[psi_start + 1], solution.x[v_start + 1],
          solution.x[cte_start + 1], solution.x[epsi_start + 1],
          solution.x[delta_start],   solution.x[a_start]};
#endif
}


#if 0
/***
 * From the MPC Quiz, the polyeval code and polyfit code
 * were in MPC.cpp.  For the MPC Project they moved to
 * main.cpp so they are not needed here.  Took them out
 * of the code.
 *
 */
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
#endif
