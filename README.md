# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---


1. MPC Project

In this project the goal is to implement a Model Predictive Control to drive the car around the track. The cross track error isn't provided by the simulator and there's a 100 millisecond latency between actuations commands on top of the connection latency.

### Following details provides steps:

- Fitting a line based on road waypoints and evaluating the current state based on that polynomial line.
- Implement the MPC calculation, including setting variables and constraints.
- Calculate actuator values from the MPC calculation based on current state.
- Account for latency.
- Calculate steering angle & throttle/brake based on the actuator values.
- Adjust Timesteps and Timesteps Duration values after testing on Udacity simulator.


2. Implementation

2.1 The Model

2.1.1 Websocket Data

This document describes the JSON object send back from the simulator command server.

Fields:

* `ptsx` (Array<float>) - The global x positions of the waypoints.
* `ptsy` (Array<float>) - The global y positions of the waypoints. This corresponds to the z coordinate in Unity
since y is the up-down direction.
* `psi` (float) - The orientation of the vehicle in **radians** converted from the Unity format to the standard format expected in most mathemetical functions (more details below).
* `psi_unity` (float) - The orientation of the vehicle in **radians**. This is an orientation commonly used in [navigation](
* `x` (float) - The global x position of the vehicle.
* `y` (float) - The global y position of the vehicle.
* `steering_angle` (float) - The current steering angle in **radians**.
* `throttle` (float) - The current throttle value [-1, 1].
* `speed` (float) - The current velocity in **mph**.

2.2 Fitting based on road waypoints and evaluating the current state

First, I transformed the points from the simulator's global coordinates into the vehicle's coordinates using standard 2d vector transformation.
Using the `polyfit()` function, a third-degree polynomial line is fit to these transformed waypoints, drawing the target path.
From the car perspective we can assume that it is always at the center of the coordinate system and always pointing to a zero orientation. The cross-track error can be calculated by evaluating the polynomial function (`polyeval()`) at px, which is now zero.Calculating epsi from the derivative of polynomial fit line is a lot easier since polynomials above the first order are all eliminated through multiplication by zero.


2.3 Model Predictive Control

MPC::Solve
The variable (state) is the initial state [x,y,ψ,v,cte,eψ], coeffs are the coefficients of the fitting polynomial. The bulk of this method is setting up the vehicle model constraints (constraints) and variables (vars) for Ipopt.

FG_eval()
First it creates cost functions for each of the variables and sets the weights to be able to influence the importance of each.
For example, lower weights related to cte and epsi lead to the model not focusing enough on staying near the center of the road and turning correctly, the (velocity) cost is there to prevent the car from stoping and the costs related to (delta) and (a), and especially the costs related to the changes of those values, are important as well. Putting more weight to (delta_change) helps the ride to be much smoother.
Finally, the updated cost constraints are calculated by first calculating the states at time t and time + 1. These states are then put through the update equations, such as the given y cost constraint being equal to y1 - (y0 + v0 * sin(psi) * dt). This, along with the variables and constraints calculated earlier, can be fed to the `ipopt` solver. This solver takes in all the information and will calculate the future predicted states, which also includes updated (delta) and (a) values that I use for my actuator values.


2.4 Latency

The simulator added 100ms latency between the actuator calculation and the time the simulator performs that action.
The best way to account for this issue was to add a step to predict where the vehicle would be after 100ms, in order to take the action that is required to actually be taken at that time, instead of the one in reaction to the data from the simulator.


2.5 Tuning Timesteps (N) and Timestep Duration (dt)

Finally, after visualizing the model in the simulator, it was time to adjust (N) and (dt). I started with values of 5 for (N) and 0.2 for (dt). The car wasn't able to stay on the track until I set (N) to 10 and the car was able to complete the track. I tried increasing even more the (N) value but the car was slowing down so I decided to leave it at 10. After toying with (dt) I noticed that 0.2 was too slow to react so I set (dt) to 0.1. Considering that after setting desired speed to 130 the car was able to complete the lap I decided to leave those variables like that.


3 Results
After increasing the desired speed to 130 and pushing the car to the limit I was delighted to see how smooth the handling was. 



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
