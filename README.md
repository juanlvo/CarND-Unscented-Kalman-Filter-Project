# Unscented Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program solution from Juan Luis Vivas Occhipinti

In this project utilize an Unscented Kalman Filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower that the tolerance outlined in the project reburic. 

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and intall [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see [this concept in the classroom](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77) for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./UnscentedKF

Note that the programs that need to be written to accomplish the project are src/ukf.cpp, src/ukf.h, tools.cpp, and tools.h

The program main.cpp has already been filled out, but feel free to modify it.

Here is the main protcol that main.cpp uses for uWebSocketIO in communicating with the simulator.


INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurment that the simulator observed (either lidar or radar)


OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

---

## Other Important Dependencies

* cmake >= v3.5
* make >= v4.1
* gcc/g++ >= v5.4

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
    - eg. `./UnscentedKF ../data/obj_pose-laser-radar-synthetic-input.txt`

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html) as much as possible.

## Generating Additional Data

This is optional!

If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.

## Project Instructions and Rubric

This information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/c3eb3583-17b2-4d83-abf7-d852ae1b9fff/concepts/f437b8b0-f2d8-43b0-9662-72ac4e4029c1)
for instructions and the project rubric.

###Compiling

<table>
	<tr>
		<th>Criteria</th>
		<th>Meets Specifications</th>
	</tr>
	<tr>
		<td>Your code should compile</td>
		<td>The code compile without errors with cmake and make.</td>
	</tr>
</table>


###Accuracy

<table>
	<tr>
		<th>Criteria</th>
		<th>Meets Specifications</th>
	</tr>
	<tr>
		<td>The project recently changed. If you developed your project with the old version, you can still turn turn in the project with the old data sets, and your review will be based on the old criteria.

For the new version of the project, there is now only one data set "obj_pose-laser-radar-synthetic-input.txt". px, py, vx, vy output coordinates must have an RMSE <= [.09, .10, .40, .30] when using the file: "obj_pose-laser-radar-synthetic-input.txt"

If you are using the older version of the project with two data sets, criteria for passing are:

that the px, py, vx, vy output coordinates have an RMSE <=[0.09, 0.09, 0.65, 0.65] when using the file: "sample-laser-radar-measurement-data-1.txt"

that the px, py, vx, vy output coordinates have an RMSE <=[0.20, 0.20, 0.55, 0.55] when using the file: "sample-laser-radar-measurement-data-2.txt" </td>
		<td>For the new data set, your algorithm will be run against "obj_pose-laser-radar-synthetic-input.txt". We'll collect the positions that your algorithm outputs and compare them to ground truth data. Your px, py, vx, and vy RMSE should be less than or equal to the values [.09, .10, .40, .30].

For the older version of the project, your algorithm will be run against "sample-laser-radar-measurement-data-1.txt". We'll collect the positions that your algorithm outputs and compare them to ground truth data. Your px, py, vx, and vy RMSE should be less than or equal to the values [0.09, 0.09, 0.65, 0.65].

For the older version, your algorithm will also be run against "sample-laser-radar-measurement-data-2.txt". The RMSE for the second data set should be <=[0.20, 0.20, 0.55, 0.55].</td>
	</tr>
</table>

###Follows the Correct Algorithm
<table>
	<tr>
		<th>Criteria</th>
		<th>Meets Specifications</th>
	</tr>
	<tr>
		<td>Your Sensor Fusion algorithm follows the general processing flow as taught in the preceding lessons.</td>
		<td>Yes the algorith fllow the steps described in the </td>
	</tr>
	<tr>
		<td>Your Kalman Filter algorithm handles the first measurements appropriately.</td>
		<td>Yes, can be found it the inititalization of the variables</td>
	</tr>
	<tr>
		<td>Your Kalman Filter algorithm first predicts then updates.</td>
		<td>Yes, like it is described in the lessons</td>
	</tr>
</table>
