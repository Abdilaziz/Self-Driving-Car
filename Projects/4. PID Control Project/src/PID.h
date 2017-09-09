#ifndef PID_H
#define PID_H

#include <vector>

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;


  // Twiddle

  int iter;
  int dp_size;
  double best_error;
  double error;
  int cur_param_index;

  std::vector<double> dp; // possible changes

  bool added_dp, subtracted_dp;


  int n_settle_steps, n_eval_steps;


  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();


  double CalcVal(int type);

  void AddChangesToParams(int index, double change );


};

#endif /* PID_H */
