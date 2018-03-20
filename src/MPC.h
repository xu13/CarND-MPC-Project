#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

struct CostWeights {
  CostWeights() {
    cte = 100;
    epsi = 100;
    v = 1;
    delta = 200;
    a = 0;
    ddelta = 0;
    da = 0;
  }

  double cte;
  double epsi;
  double v;
  double delta;
  double a;
  double ddelta;
  double da;
};

struct Cost {
  Cost() {
    cte = 0;
    epsi = 0;
    v = 0;
    delta = 0;
    a = 0;
    ddelta = 0;
    da = 0;
  }

  double cte;
  double epsi;
  double v;
  double delta;
  double a;
  double ddelta;
  double da;
};

class MPC {
public:
  MPC();

  virtual ~MPC();



  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  std::vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

};

#endif /* MPC_H */
