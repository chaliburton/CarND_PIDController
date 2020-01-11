#include "PID.h"
#include <cmath>
#include <iostream>
#include <algorithm>
/**
 * PID Class
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * Initialize PID coefficients (and errors, if needed)
   */
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  p_error = 0;
  i_error = 0;
  d_error = 0;
}

void PID::UpdateError(double cte) {
  /**
   * Update PID errors based on cte.
   */
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;
  t_error = d_error + p_error + i_error;
}

double PID::TotalError() {
  /**
   * Calculate and return the total error
   */
  return -Kp*p_error - Ki*i_error - Kd*d_error;  // TODO: Add your total error calc here!
}

double PID::GetP() {
  /**
   * Return private member P
   */
  return Kp;
}

double PID::GetI() {
  /**
   * Return private member I
   */
  return Ki;
}

double PID::GetD() {
  /**
   * Return private member D
   */
  return Kd;
}

double PID::GetT() {
  /**
    * Return privte member total error
    */
   return t_error;
}
