# CarND-Controls-MPC

Please check this link for [video](https://youtu.be/-Vq2laksfrU)

The control runs much faster than previous submission with better performance of maintaining steady state.

There are a couple of issues with current simulator and vehicle physics model setup.

 1. Relation of throttle and acceleration was not considered
 2. Converting velocity from miles per hour (mph) unit used by Unity to SI unit makes model unstable.
 3. Normalizing steer angle to -1, 1 makes model unstable

Kinematics Model
--

I am using standard kinematics model derived as in classroom. 

![kinematics_model](./kinematics_model.png)

TODO:
A more accurate second order (dynamics) model could be used to account orientation difference
caused by acceleration.

Cost function and optimization
---
![Cost function](./cost_func.png)

Look ahead time is set as T = N (10) * dt (0.1) = 1s. As vehicle target speed set in 100mph. N = 25, N = 20, N = 16 are used before, as experiments shown, N = 10 has similar effect with larger N number when total look ahead time Ts are roughly same.
Large cost weights are assigned to orientation error, large steering angle and large steering input change within unit sample interval.
To achieve smooth, responsive steering control. 

In first iteration, unity velocity unit miles per hour was converted to SI unit meter per second. However, when working on improve model performance. This approach leads to car heavily wobbling from one side of track to another. Unit conversion may change state progression
 model calculation, which leads to different landscape of cost function Hence, in current submission to achieve stable model prediction. We use miles per hour unit for velocity penalty.

In this project, the MPC problem is essentially posed as receding horizontal optimization problem. Ipopt is a library read as interior points optimization.
The name suggests interior points methods which are a certain class of algorithms widely used in linear and 
 nonlinear optimization problem. After poking around Ipopt library, it seems internally it is using convex optimization rules for calculating
 optimal points. KKT condition is also mentioned in one of debugging output. The cost function itself is also in a form of 
 SOS (sum of square) of quadratic functions which have quite a few interesting properties worth learning.
 
 * TODO: Add curvature based velocity weight terms
 * TODO: Read more about convex/non-convex optimization, interior points method, KKT rules
 * TODO: Read more about SOS (sum of square) quadratic cost functions, regularization and other related topics

Dealing with latency
---
In this project, we need to counter measure latency. Because predictive nature of latency and 
assumption of perfect kinematics model without disturbance. The solution is relatively naive.
To account for latency, we predict the vehicle state in the future using vehicle dynamics introduced above before passing it to the solver.
Then take the actuator output at the moment for future control input as the latency is modeled after
actuator delay.

Coordinate transformation
---
TODO: transform against delay  

Navigation way points are defined in global/unity coordination. For ease of calculation and visualization,
global navigation way point is transformed into local/vehicle coordination.

<img src="https://cdn-enterprise.discourse.org/udacity/uploads/default/original/4X/3/0/f/30f3d149c4365d9c395ed6103ecf993038b3d318.png" width="270"/>


[Forum discussion](https://discussions.udacity.com/t/do-you-need-to-transform-coordiantes/256483)

Reading List
---
[Model-Predictive
 Motion Planning](http://people.csail.mit.edu/rak/www/sites/default/files/pubs/HowEtal14.pdf)

---

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
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
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
