# CarND-Controls-MPC Project Solution
Self-Driving Car Engineer Nanodegree Program

My controller accepts three parameters: the first one is dt - simulation time step, the second one is ref_v - speed in miles per hour, and the last one is dt_run - time step used to calculate steering of a car during runtime. I use N=25 for the number of simulation time steps. This translates into simulated time of N * dt = 1.25 seconds in the first case below, 1.075 seconds in the second case, and 1.05 seconds in the last case, when I was running the car at 80 mph. Note that for the first and the second cases the simulation time step is equal to the runtime time step, which is what you would typically expect. At higher speeds, I found that the runtime step could be lower than the simulation time step, which means that the car tries to understeer at high speeds.    

I am fitting the polynomial of the second degree to my waypoints. I find it sufficient to describe the curves we are encountering. Waypoints are processed in the following way: they are rotated relative to location of the car in the direction opposite the vehicle orientation psi prior to fitting the polynomial. This transformation avoids the problems of fitting the polynomial of the second degree to the untransformed points. Simulation is done from the point of view of the vehicle, so x, y and psi components of the initial state are all 0, cross-track error cte is effectively C0 of the polynomial, vehicle speed v is untransformed and orientation error epsi is -arctan(C1), where C1 is the second coefficient of the polynomial. I am using the standard state for the MPC, which includes coordinates of the car x and y (in the coordinate system of the car itself), orientation psi, speed v, cross-track error cte and orientation error epsi. 

I use the standard update equations for the state:
```
x = x0 + v0 * cos(psi0) * dt
y = y0 + v0 * sin(psi) * dt
psi = psi0 + v0 * delta0 / Lf * dt
v = v0 + a0 * dt
cte = (f0 - y0) + v0 * sin(epsi0) * dt
epsi = psi0 - psides0 + v0 * delta0 / Lf * dt
```
Where f0 is a polynomial of the second degree:
```
f0 = C0 + C1 * x0 + C2 * x0^2
```
and psides0 is the arctangent of the derivative of f0:
```
psides0 = arctan(C1 + 2 * x0)
```

I calculate steering the following way (note that the use of parameter dt_run instead of dt allows me to understeer at higher speeds, which leads to a more stable performance):
```
steer_value = - (v1*dt_run*delta2/Lf)/deg2rad(25);
```
Finally, throttle is just equal to acceleration a2.

My controller's solver returns all 24 x and y coordinates of the predicted trajectory, psi1, v1, cte1, epsi1 and delta2 and a2. Note that I handle a 100 millisecond latency by using delta2 and a2, instead of delta0 and a0 - meaning that I am looking two simulation steps ahead for the values of delta and a. Since dt is the first case below is 0.05, this translates into exactly 100 milliseconds delay in applying actuators. In the second case below delay is about 86 milliseconds due to lowering of dt and in the third case it is 84 milliseconds. 

Here is the simulated car driving at a speed of 60 mph controlled by MPC controller.
I used the following command line: 
```
./mpc 0.05 60 0.05
```
[![Car driving at 60 mph on the first track controlled by MPC controller](https://i.ytimg.com/vi/S2a39caBX70/2.jpg)](https://youtu.be/S2a39caBX70)

Here is the simulated car driving at top speed of 80 mph controlled by MPC controller.
I used the following command line: 
```
./mpc 0.043 70 0.043
```
[![Car driving at 70 mph on the first track controlled by MPC controller](https://i.ytimg.com/vi/s-5VzlqTZec/3.jpg?)](https://youtu.be/s-5VzlqTZec)

Here is the simulated car driving at top speed of 80 mph controlled by MPC controller.
I used the following command line: 
```
./mpc 0.042 80 0.037
```
[![Car driving at 80 mph on the first track controlled by MPC controller](https://i.ytimg.com/vi/QS7i90OQsjo/2.jpg)](https://youtu.be/QS7i90OQsjo)
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
4. Run it: `./mpc 0.05 60 0.05`.

Note, that the first parameter to the controller is dt - the time step of the simulation, the second is speed in miles per hour and the third is dt_run - the time step used during steering calculation, which for low speeds you can set to be equal to dt, but at higher speeds it is recommended to set it lower to improve stability of the car.
