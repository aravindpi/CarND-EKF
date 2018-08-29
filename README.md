# Extended Kalman Filter Project Code
Self-Driving Car Engineer Nanodegree Program

In this project I utilized an extended kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. I also obtained RMSE values that are lower than the tolerance outlined in the project rubric. 

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

## Source Code
The primary files are src/FusionEKF.cpp, src/FusionEKF.h, kalman_filter.cpp, kalman_filter.h, tools.cpp, and tools.h. The header files were provided as a starting point. I implemented the source (cpp) files.

INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)

OUTPUT: values provided by the c++ program to the simulator

["px"] <= kalman filter estimated position x
["py"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `

## Results

*Data Set 1*

| Input | MSE    |
| ----- | -------|
|  px   | 0.0973 |
|  py   | 0.0855 |
|  vx   | 0.4638 |
|  vy   | 0.4345 |



*Data Set 2*

| Input | MSE    |
| ----- | -------|
|  px   | 0.0729 |
|  py   | 0.0968 |
|  vx   | 0.4099 |
|  vy   | 0.4892 |




