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
  throt_tgt = 0;
  CTE_n_prev = 0;
  
}

void PID::UpdateError(double cte) {
  /**
   * Update PID errors based on cte.
   */
  prev_cte = p_error;
  d_error = cte - prev_cte;
  p_error = cte;
  i_error += cte;


}

double PID::TotalError() {
  /**
   * Calculate and return the total error
   */
  return -Kp*p_error - Ki*i_error - Kd*d_error;  // TODO: Add your total error calc here!
}

/*double PID::SetSpeed(double speed, double CTE_n, double angle) {
  /**
   *  Set speed limit based on steering robustness
   *//*
  if(std::abs(speed)<20) {
	throt_tgt = 0.25; 
  } else if( (CTE_n-CTE_n_prev) > 0.5) {
	throt_tgt -= 0.2;
  } else {
	throt_tgt += 0.0025;
	throt_tgt = std::min(0.70,throt_tgt);
  }
  CTE_n_prev = CTE_n;
  return throt_tgt;
}*/


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
