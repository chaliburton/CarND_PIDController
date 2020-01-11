# [Project - PID Controller]
Self-Driving Car Engineer Nanodegree Program
Chris Haliburton

This project can also be found https://github.com/chaliburton/CarND_PIDController/

In this project I was provided with initial code that read certain parameters from a vehicle simulator.  The Center Tracking Error, speed, angle are sent from the vehicle simulator.  The CTE is calcualted from the d= 0 value of Frenet coordinates.  The starter code precluded functionality of the steering and output a constant speed value in main.cpp.

[main.cpp]
This code was provided to setup the interface messaging with the simulator.  Functions and variables were added to achieve lateral vehicle motion and longitudinal speed control.  Two instances of the PID class were instantiated to allow for side to side lateral control via the steering output and the second instance was for vehicle target speed control via throttle position.  The throttle position / torque request is not a linear speed control method.

Within main.cpp the elements are instantiated and the first main task is to establish a speed target.  The significant challenge with tuning a steering PID is that as speed increases, the sampling frequency stays the same so the vehicle can be further from it's desired point and a stronger control response is required to correct.  A lower speed allows for more frequent corrections.  The same PID was used to control steering and was initially tuned using a throttle output of 0.3.  Once it was semi-stable the speed target vs steering angle code was implemented.

The speed target vs steering angle ensured that as the steering angle increased in either direction the speed target decreased such that the vehicle would brake to help recover to the center of the track.

The speed error was calculated and a pid was implemented to apply more throttle or brake.  The velocity pid was not tuned, a large proportional gain was used to quickly brake the vehicle, a very, very small integral gain was used as speed was not critical to this project and a small derivative gain was used which had limited impact other than to smooth braking (this would assist in wheel slip/traction loss if that was modelled by the simulator)

The simulator output CTE, the cross track error from d = 0 and this was used to update the pid error.  The steering value was then found from the Kp, Ki, Kd values and each terms error.

TWIDDLE was implemented to hunt for the best Kp, Ki and Kd values for the steering pid.  The length of runtime for sampling error was small to begin with and then I increased it to get more curves into the learning function.  After each runtime the simulator was reset to the beginning and the steering pid was reinitalized to ensure comparable learning conditions for each run.  The TWIDDLE algorithm was adopted from the one discussed in class. Once the TWIDDLE algorithm learning parameters were sufficiently small the TWIDDLE algorithm printed the final parameters.

The duration of the runtime was increased to the point where the vehicle was actually learning over the period of an entire lap.  The speed of the vehicle was able to increase such that the distance travelled by the end travelled much farther than the beginning.

# [For the Project Rubric:]
The code compiles without error.

[The PID procedure follows what was taught in the lessons..]
TWIDDLE was implemented to tune the PID controller.  Two PID controllers were implemented, one for steering and one for speed.

[Describe the effect each of the P, I, D components had in your implementation.]
The P- gain works to cut the error down quickly.  For the steering angle this helps to quickly correct an error (ie on center tracking suddenly into a tangent curve and the CTE rapidly increases.)  For the speed this quickly increases or decreases speed.

The I- gain works to eliminate the error over longer periods of time.  For the steering angle this helps to reduce bias in the steering hardware (alignment or improper tire inflation)  the integral adds up the error and over time and helps to correct this.  For the speed control this was very very small as steady state speeds were rarely encountered on the race track course.

The D- gain works to help reduce overshoot/overcorrection by limiting the rate at which the controller corrects the error.  For the steering angle this helps to reduce oscillations around the center point caused by misalignment of the vehicles Y-axis projection along the S-axis of the road.  For the speed controller this works to approach the target speed more smoothly.  On the race track course this has limited impact due to the amount of turning and constant speed changes.

[Describe how the final hyperparameters were chosen.]
TWIDDLE was implemented as described above.  I let this run for nearly five hours resetting and starting a new lap each iteration.

[The vehicle must successfully drive a lap around the track.]
The vehicle is capable of driving safely around the track.
