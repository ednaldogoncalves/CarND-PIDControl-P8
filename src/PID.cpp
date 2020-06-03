#include "PID.h"
#include <algorithm>
/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
	PID::Kp = Kp; // Proportionnal term
	PID::Ki = Ki; // Integral term 
	PID::Kd = Kd; // Differential term

	// Init the Proportionnal error to 0
	p_error = 0.0;

	// Init the Integral term to 0
	// Because, at the very beginning, the total area (Integral definition) between the position of the car 
	// and the CTE is null (because the car has not started yet)
	i_error = 0.0;

	// Init the Differential error to 0
	// Because, at the very beginning, the position of the car is assumed to be far away from CTE
	d_error = 0.0;
	
	// Previous cte.
	prev_cte = 0.0;
	
	// Counters.
	count = 0;
	errorSum = 0.0;
	minError = std::numeric_limits<double>::max();
	maxError = std::numeric_limits<double>::min();
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
   
	// Proportional error.
	p_error = cte;

	// Integral error.
	i_error += cte;

	// Diferential error.
	d_error = cte - prev_cte;
	prev_cte = cte;

	errorSum += cte;
	count++;

	if ( cte > maxError ) {
		maxError = cte;
	}
	if ( cte < minError ) {
		minError = cte;
	}
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  return p_error * Kp + i_error * Ki + d_error * Kd;
}

double PID::AverageError() {
  return errorSum/count;
}

double PID::MinError() {
  return minError;
}

double PID::MaxError() {
  return maxError;
}
