# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

# Overview
This repository contains all of the code implemented for the PID Controller assignment in Udacity's Self-Driving Car Nanodegree.

## Submission
A completed version of [PID.cpp](./src/PID.cpp) was implemented in C++ and is located in the [./src](./src/) directory. The controller operates with vehicle's cross-track error and speed at each time step and outputs a steering command to minimize the error and hold the vehicle on line. 

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros)
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Compiling and executing the project code

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## [Rubric](https://review.udacity.com/#!/rubrics/824/view) points

### Compilation: 
The implementation including [main.cpp](./src/main.cpp), [PID.h](./src/PID.h) and [PID.cpp](./src/PID.cpp) compiles without error.

### Implementation: 
The code implements the basic PID controller in [PID.cpp](./src/PID.cpp). The main function UpdateError updates the controller error terms with the value of the current cross track error:

```
            //Calculate P_error, I_error and D_error from current cte, last cte and delta time
            p_error = cte;

            i_error += cte * dt;

            //Limit the integral term winding up
            if(i_error > 10.0)
            {
                i_error = 10.0;
            }
            else if(i_error < -10.0)
            {
                i_error = -10.0;
            }

            d_error = (cte - last_cte) / dt;
```
The p_error term represents the current cross track error.
The i_error term is an accumulation of steady state error over time.
The d_error term is the current rate of change of cross track error.
The output from UpdateError is a steering command calculated as following:

```
    double pid_output = -( (Kp * p_error) + (Ki * i_error) + (Kd * d_error) );
```
The coefficients Kp, Ki and Kd represent gains for the p_error, i_error and d_error terms.

### Reflection: 
Compensation for the p_error term eliminates the vehicle cross-track error. Compensation for the d_error term (rate of change of cross-track error) is to reduce oscillation caused by overshoot of p_error compensation. Compensation for the i_error term is to reduce accumulated steady state error caused by vehicle steering's bias and other uncertainty of the system.

The implementation also considers vehicle current speed. For a given constant error of steering command, the vehicle is getting away from the desired trajectory with an acceleration directly proportional to the squared of the vehicle speed. Subsequently vehicle's cross track error after a fix period of time dT also directly proportional to the squared speed. To account for speed change, the output from the PID controller is scaled down by the amount of squared speed. It makes the steering command output from the controller independent from the speed of the vehicle. 

The final steering command is also range limited between -1.0 and 1.0. 

A minimum speed (10 mph) is also used for PID output scaling to stop the fluctuation of steering command at low speeds.

The code of the above mentioned in [main.cpp](./src/main.cpp) is following:
```
          double speed = std::stod(j[1]["speed"].get<std::string>());

          //Set a minimum speed used for scaling PID output
          const double MIN_TUNED_SPEED = 10.0;
          if(speed < MIN_TUNED_SPEED)
          {
              speed = MIN_TUNED_SPEED;
          }

          //Scaling PID output by squared speed)
          double steer_value = pid.UpdateError(cte) / (speed * speed);

          //Limit steer value between -1.0 and 1.0
          if(steer_value >  1.0)
          {
              steer_value =  1.0;
          }
          else if(steer_value < -1.0)
          {
              steer_value = -1.0;
          }

```
The hyperparameters (PID gains) Kp, Ki and Kd were first picked manually.
First the Kp gain was selected to roughly compensate for the vehicle cross track error running a full lap.
Next the Kd gain was picked to reduce the oscillation caused by the P-term compensation.
Last the Ki gain was chosen to reduce the effect of steering bias.
The result of manual selection was 

Kp = 250 

Ki = 25 

Kd = 2.5 

With the above PID gains the vehicle could complete a lap of simulation with an averaged squared error of about 0.3 at a speed of 33 mph.

The PID controller also implemented an optimisation method RunOptimisation based on Twiddle from the lesson. The manual selected gains were set to be the initial value for Twiddle optimisation. The gain steps were set to be 20 for Kp, 5 for Ki and 1 for Kd. The total gain step tolerance value was set to 1. The number of updates of cross track error was set to 1125 to approximately cover for the whole lap.

The result of running Twiddle parameter optimisation is captured in [test_run_optimisation.txt](./results/test_run_optimisation.txt). The result after running 22 iterations and 137 laps of simulation was 

Kp = 213.962

Ki = 33.0595

Kd = 2.5

Result of a test run with optimized gains is captured in [test_run_best_gains.txt](./results/test_run_best_gains.txt). The averaged squared cross track error was about 0.245173 at 33 mph.

### Simulation:

The vehicle successfully completed a lap around the track with no tyre leaving the drivable portion of the track surface. The vehicle did not pop up onto ledges or roll over unsafely.

A full simulation run was captured in the [test_result.mp4](./results/test_result.mp4)






