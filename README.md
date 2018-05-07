### **MPC controller**

## Writeup
---

### Udacity Course, October 2017 cohort

**Self-Driving Car Engineer Nanodegree Program**

**Project 'MPC Controller', May 2018**

**Claus H. Rasmussen**

---

# Build a Model Prediction Controller in C++ and maneuver the vehicle round the lake track in the Udacity simulator.

**The Model**
The Model Prediction Controller is build around the concept of 'simple' kinematic models that ignore tire forces, gravity, and mass. The model has a state consisting of position *(x,y)*, an orientation angle *(psi)* and a velocity *(v)*. The implemented model tracks the evolving state over time and two actuators/control inputs, steering angle *(d)* and throttle *(a)*, that allows us to control the vehicle state.

At time t+1, where dt is the time step, the formulas for calculating the new state are:
```cpp
x(t+1)   = x(t) + v(t) * cos(psi) * dt
y(t+1)   = y(t) + v(t) * sin(psi) * dt
psi(t+1) = psi(t) + v(t) * d(t) * dt / Lf
v(t+1)   = v(t) + a(t) * dt
```
The simulator sends a trajectory in form of a series of waypoints to the model, that fits them to a 3rd order polynomial, using Eigen. Given a fixed duration and timesteps (see next section), a number of discrete paths between the actuations to be used by the car, makes the car following the predicted trajectory. A cost function is used to ensure that the actuators receive values, that makes the car drive as good as possible, making the errors between the actual path and the predicted as small as possible.


**Elapsed Duration and Timestep Length (N & dt)**
With a value for Timestep Length fixed at **0.1** seconds, I tried out a couple of values for Elapsed duration (N) and found that the car crashed using a value of 20, while setting N to **10** (equals to a time horisont of one second), the car is able to do several lapses around the lake track.
I believe it is possible to fine tune *N* and *dt* a little further, e.g. N=14 and dt=0.08 may give a better overall result, but I haven't gone deeper into this in this project.


**Polynomial Fitting and MPC Preprocessing**
The waypoints, send from the simulator, are processed with the provided *polyfit(...)* function in main.cpp, line 111. The result is the coefficients to the fitted polynomial, which first are used to calculate the cross track error and the orientation error values, *cte* and *epsi*, again using a provided function, *polyeval(...)* in main.cpp, line 114 and 117. After calculating the state vector, considering a latency of 100 milliseconds, the coefficients are used in the central MPC.Solve(...) function, along with the state vector (main.cpp, line 155).

The MPC object, from MPC.cpp and MPC.h, is set up using a cost function, in MPC.cpp, line 53-74. This happens in a *FGeval* object, that is used by the MPC.solve(...) function. In the *FGeval* object, the errors (*cte* and *epsi*) are added to the cost function using a high value multiplicator (= 2000) to ensure that these variables gets 'high attention'. Furthermore values in the power of two for speed, steering angle and throttle values are added, including a 'difference cost' for steering and throttle, in order to make sure that they don't grow or shrink too fast during the drive. Next the constraints are added to the *fg* vector.

In the MPC.Solve(...) function (MPC.cpp, line 134-268), the upper and lower limits for the variables for the states and actuators are set. Values for the initial state are added last. The are comments in the code for these variables.

After solving the problem, using the function CppAD::ipopt::solve(...), MPC.cpp, line 238, the result is processed before it is returned to the simulator and used to actuate steering and throttle.


**Model Predictive Control with Latency**
In main.cpp, lines 126-145, a latency (delay_t = 100 milliseconds) is added to the state vector:
```cpp
if (consider_latency) {
  // State values reflects latency
  double delay_x = v * delay_t;
  double delay_y = 0;
  double delay_psi = v * -steer_value / Lf * delay_t;
  double delay_v = v + throttle_value * delay_t;
  double delay_cte = cte + v * sin(epsi) * delay_t;
  double delay_epsi = epsi + v * -steer_value /Lf * delay_t;
  state << delay_x, delay_y, delay_psi, delay_v, delay_cte, delay_epsi;
}
else {
  // State values without latency corrections
  state << 0, 0, 0, v, cte, epsi;
}
```
---

And it eventually was a success, it took **a lot** of debugging and testing, sometimes it was fun, but it turned out well with a car that actually drives quite nicely :-)

---
---
---

## **This section was provided by Udacity to help set up the coding environment etc.**

# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
