#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
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
   * TODO: Update PID errors based on cte.
   */
  d_error = p_error-cte;
  p_error = cte;
  i_error += cte;


}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  return Kp*p_error + Ki*i_error + Kd*d_error;  // TODO: Add your total error calc here!
}

double PID::SetSpeed(double speed, double angle) {
  /**
   *  Set speed limit based on steering robustness
   */
  
  if (std::abs(p_error >.5) {
	speed -= 3.5;
  }
  if (std::abs(angle>5) {speed -= 1.0;
	if (std::abs(angle>10) {speed -= 2.5;  
		if (std::abs(angle>15) {speed -= 2.5;  
			if (std::abs(angle>20) {speed -= 5;  
				if (std::abs(angle>25) {speed -= 5;
				}
			}
		}
	}
  } else { speed = std::min(100,speed += 0.25);
  }    
  

  std::cout<<"                   			Speed now " <<speed; 
  return speed;
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
