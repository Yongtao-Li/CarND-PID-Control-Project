#include "PID.h"
#include <iostream>

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
  dKp = 1.0;
  dKi = 1.0;
  dKd = 1.0;
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  return -1.0 * Kp * p_error - 1.0 * Ki * i_error - 1.0 * Kd * d_error;
}

void PID::KpInc() {
  Kp += dKp;
}

void PID::KiInc() {
  Ki += dKi;
}

void PID::KdInc() {
  Kd += dKd;
}

void PID::KpDec() {
  Kp -= 2.0*dKp;
}

void PID::KiDec() {
  Ki -= 2.0*dKi;
}

void PID::KdDec() {
  Kd -= 2.0*dKd;
}

void PID::dKpInc() {
  dKp *= 1.1;
}

void PID::dKiInc() {
  dKi *= 1.1;
}

void PID::dKdInc() {
  dKd *= 1.1;
}

void PID::dKpDec() {
  dKp *= 0.9;
}

void PID::dKiDec() {
  dKi *= 0.9;
}

void PID::dKdDec() {
  dKd *= 0.9;
}

void PID::piddebug(){
  std::cout << "Kp = " << Kp << " Ki = " << Ki << " Kd = " << Kd << std::endl;
  std::cout << "dKp = " << dKp << " dKi = " << dKi << " dKd = " << dKd << std::endl;
  std::cout << "p_error = " << p_error << " i_error = " << i_error << " d_error = " << d_error << std::endl;
}

void PID::ResetError() {
  /**
   * TODO: Update PID errors based on cte.
   */
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
}

bool PID::ResetTwiddle(double tol) {

  if (dKp+dKi+dKd < tol)
  {
    std::cout << "twiddle ended successfully!" << std::endl;
    return false;
  } else
  {
    return true;
  }
  
}