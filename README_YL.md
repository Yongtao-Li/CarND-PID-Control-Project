# **PID Controller Project**

## Yongtao Li

---

The goals of this project are the following:

* Implement a PID controller in C++ to maneuver the vehicle around the track

## Rubric Points
### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/1972/view) individually and describe how I addressed each point in my implementation.

---

#### 1. Compilation

All the source code compile and run successfully on my local machine. Since I'm using Windows10, I chose to setup the environment to run Ubuntu Bash on Windows.

#### 2. Implementation

The PID procedure is implemented as following. During each update step, the PID error terms are getting updated and the total error is updated based on current PID parameters.

```C
void PID::UpdateError(double cte) {
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;
}

double PID::TotalError() {
  return -1.0 * Kp * p_error - 1.0 * Ki * i_error - 1.0 * Kd * d_error;
}
```

#### 3. Reflection

##### 3.1 The effect of each P, I, D components

* The P component sets the steering angle in proportion to CTE value. If the car is far to right, then it will try to steer to left and vice versa. However with only P component, the car would steer back and forth with overshoot.

* The D component sets the steering angle in proportaion to the difference between current CTE and previous CTE. It helps reduce overshoot.

* The I component would sets the steering angle in proportaion to the accumulated CTE over the course. It helps reduce systematic bias, which PD controller couldn't correct. In this project, the final parameter for I component is zero, probably because there isn't any systematic bias built in the simulator.

##### 3.2 How the final hyperparameters were chosen

The Twiddle algorithm has been implemented to tune the PID parameters. The final PID parameters are (0.416933, 0.0, 2.03224). There are a few lessons learned as following.

* Unlike the Python code for Twiddle, the C++ code need to calculate error on each time step and update PID parameters when a smaller error is found.
* It's better to tune the parameters over a longer time peorid (2000 steps), therefore having error comparison at every step and allow early exit will speed up the tuning.
* Throttle control is also useful. I think when the car is going too left or right on a fast speed, we might want to slow down a little bit to get over the turn.

#### 4. Simulation

As you could see from the following video, the car could drive around the track using the parameters tuned from Twiddle. The car isn't pop up onto any edges or roll over undrivable surfaces.

<a href="https://youtu.be/0_3L46O5N8g
" target="_blank"><img src="http://img.youtube.com/vi/0_3L46O5N8g/0.jpg" 
alt="High Way Driving" width="240" height="180" border="10" /></a>

