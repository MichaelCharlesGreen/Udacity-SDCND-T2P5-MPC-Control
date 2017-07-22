# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Rubric
### Compilation
Code must compile without error with `cmake` and `make`.
### Implementations
#### The Model
Describe the model in detail. This includes the state, actuators and update equations.

Kinematic models are simplifications of dynamic models that ignore tire forces, gravity, and mass. This simplification reduces the accuracy of the models, but it also makes them more tractable.

At low and moderate speeds, kinematic models often approximate the actual vehicle dynamics.

The vehicle's state is captured by its position in coordinate space (x,y), its orientation (psi), and its velocity (v):

[//]: # (Image References)
[actuators]: ./images/actuators.png
[cross_track_error]: ./images/cross_track_error.png
[new_state_vector]: ./images/new_state_vector.png
[orientation_error]: ./images/orientation_error.png
[state_update_equations]: ./images/state_update_equations.png
[state_vector]: ./images/state_vector.png

![state vector image][state_vector]

The actuators [delta (steering angle), acceleration (a)], or control inputs, to our system:

![actuators image][actuators]

Model how the vehicle state evolves through time:

![state update equations image][state_update_equations]

Model the cross track error (CTE):

![cross track error image][cross_track_error]

Model the orientation error:

![orientation error image][orientation_error]

The new vehicle state:

![new state vector image][new_state_vector]

#### Timestep Length and Elapsed Duration (N & dt)
Discuss the reasoning behind the chosen *N* (timestep length) and *dt* (elapsed time between timesteps) values. Additionally, detail the previous values tried.

The prediction horizon is the duration over which future predictions are made; it is the product of *N* and *dt*. *N* is the number of timesteps in the horizon. *N* also determines the number of variables optimized by the MPC. *dt* is how much time elapses between actuations.

The values chosen for *N* and *dt* are 10 and 0.1 respectively. These were chosen based on the livestream demonstration and through the lessons, which state that *N* should be as large as possible while considering that it is a computational cost and *dt* should be as small as possible.

In the case of driving a car, the prediction horizon (*N* * *dt*) should be a few seconds, at most. Beyond that horizon, the enviornment will change enough that it won't make sense to predict any further into the future.

The goal of the Model Predictive Control is to optimize the control inputs. An optimizer will tune these inputs until a low cost vector of control inputs is found. The length of this vector is determined by *N*. Thus, *N* determines the number of variables optimized by the MPC.

MPC attempts to approximate a continuous reference trajectory by means of discrete paths between actuations. Larger values of *dt* result in less frequent actuation, which makes it harder to accurately approximate a continuous reference trajectory. This is sometimes called "discretization error".

A good approach to setting *N*, *dt*, and the prediction horizon is to first determine a reasonable range for the prediction horizon and then tune *dt* and *N* appropriately, keeping the effect of each in mind.

#### Polynomial Fitting and MPC Preprocessing
A polynomial is fitted to waypoints.

If the waypoints are preprocessed, the vechicle state, and/or actuators prior to the MPC procedure, it is described.

The waypoints are transformed to the vehicle's coordinate system (main.cpp lines 96-102). Because the vehicle's x and y coordinates and the orientation angle are at the origin and 0, it will be easier to fit a polynomial to the waypoints, the polynomial that provides a target path for the vehicle.

#### Model Predictive Control with Latency
Implement Model Predictive Control that handles a 100 millisecond latency. Provide details on how latency is dealt with.

*dt* is 100ms which is the same as the latency. The kinematic equations depend on the actuations from the previous timestep (*dt*). Considering latency of 100ms, the kinematic equations have to be altered since the actuations are being applied an additional timestep (*dt*) later (MPC.cpp lines 177-180).

The lessons suggested several cost functions involving minimizing: CTE, epsi, velocity compared to a reference velocity, delta (acceleration), change in delta (change in acceleration). An additional cost function was implemented using velocity and acceleration to affect cornering (MPC.cpp lines 99-125).

Note the multipliers used in the cost functions to tune the MPC.

### Simulation
The vehicle must successfully drive a lap around the track. No tire may leave the drivable portion of the track surface. The car may not pop up onto ledges or roll over any surfaces that would otherwise be considered unsafe (if humans were in the vehicle).

The car can't go over the curb, but, driving on the lines before the curb is ok.

[Here](https://youtu.be/S1-ZBVvbJ8E) is a recording showing the vehicle successfully transversing the Simulator track.

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
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
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `sudo bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
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
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

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
