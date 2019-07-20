#ifndef PID_H
#define PID_H

class PID {
 public:
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
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_);

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

  /**
   *update PID coeff
   */
  void KpInc();
  void KiInc();
  void KdInc();
  void KpDec();
  void KiDec();
  void KdDec();
  void dKpInc();
  void dKiInc();
  void dKdInc();
  void dKpDec();
  void dKiDec();
  void dKdDec();

  /**
   *print for debug
   */
  void piddebug();

  /**
   *reset errors
   */
  void ResetError();

  /**
   *turn off twiddle
   */
  bool ResetTwiddle(double tol);

 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;
  double dKp;
  double dKi;
  double dKd;
};

#endif  // PID_H