#ifndef PID_H
#define PID_H

class PID {
 public:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;
  double prev_cte;  

  /**
   * Error counters
  **/
  long count;
  double errorSum;
  double minError;
  double maxError;

  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;
  
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp, Ki, Kd) The initial PID coefficients
   */
  void Init(double Kp, double Ki, double Kd);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();
  
  /*
  *  Returns the average error.
  */
  double AverageError();

  /*
  * Returns the min error.
  */
  double MinError();

  /*
  * Returns the max error.
  */
  double MaxError();
};

#endif  /* PID_H */