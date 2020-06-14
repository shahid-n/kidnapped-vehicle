# Overview
The goal of this project is to perform 2-D localisation by means of the particle filter approach.

#### Submission
The main project submission is the following repository link: [Kidnapped Vehicle Project](https://github.com/shahid-n/kidnapped-vehicle).

Within this repository, the localisation algorithm has been implemented in the file [particle_filter.cpp](https://github.com/shahid-n/kidnapped-vehicle/blob/master/src/particle_filter.cpp).

## Project Introduction
Our autonomous robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial position, and lots of (noisy) sensor and control data.

In this project we have implemented a two dimensional particle filter in C++. The particle filter shall be given a map and some initial localisation information (analogous to what a GPS would provide). At each time step the filter will also get observation and control data.

[//]: # (Image References)

[run1]: ./output/pass_100p.png "Output from the first run: 100 particles"
[run2]: ./output/pass_1000p.png "Output from the second run: 1000 particles"

## Project Output
The project was compiled and run with the following two parameter values: 100 and 1000 particles, respectively.

The figure below shows the simulation end result with 100 particles.

![alt text][run1]

Next, the following figure shows the result of creating 1000 particles.

![alt text][run2]

Both simulations passed on the author's local machine after meeting all the evaluation criteria. In order to ensure the best chances of success on a wide variety of platforms, the submitted code has this parameter set to 20 particles, which also passes locally (this simulation result is not included, however).

---
## Running the Code
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

This repository includes two files that can be used to set up and install uWebSocketIO for either Linux or Mac systems. For windows one can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main programme can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter

Alternatively some scripts have been included to streamline this process; these can be leveraged by executing the following in the top directory of the project:

1. ./clean.sh
2. ./build.sh
3. ./run.sh

Tips for setting up the local environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

Note that the files which define the core localisation algorithm are [src/particle_filter.cpp](https://github.com/shahid-n/kidnapped-vehicle/blob/master/src/particle_filter.cpp), and [particle_filter.h](https://github.com/shahid-n/kidnapped-vehicle/blob/master/src/particle_filter.h).

The programme main.cpp had already been filled out to facilitate automated grading/evaluation of the project.

Below is a summary of the main protocol that `main.cpp` uses for uWebSocketIO in communicating with the simulator.

INPUT: values provided by the simulator to the C++ programme

// sense noisy position data from the simulator

["sense_x"]

["sense_y"]

["sense_theta"]

// get the previous velocity and yaw rate to predict the particle's transitioned state

["previous_velocity"]

["previous_yawrate"]

// receive noisy observation data from the simulator, in a respective list of x/y values

["sense_observations_x"]

["sense_observations_y"]


OUTPUT: values provided by the c++ programme to the simulator

// best particle values used for calculating the error evaluation

["best_particle_x"]

["best_particle_y"]

["best_particle_theta"]

//Optional message data used for debugging particle's sensing and associations

// for respective (x,y) sensed positions ID label

["best_particle_associations"]

// for respective (x,y) sensed positions

["best_particle_sense_x"] <= list of sensed x positions

["best_particle_sense_y"] <= list of sensed y positions


# Implementing the Particle Filter
The directory structure of this repository is as follows:

```
root
|   build.sh
|   clean.sh
|   CMakeLists.txt
|   README.md
|   run.sh
|
|___data
|   |   
|   |   map_data.txt
|   
|   
|___src
    |   helper_functions.h
    |   main.cpp
    |   map.h
    |   particle_filter.cpp
    |   particle_filter.h
```

## Inputs to the Particle Filter
The inputs to the particle filter algorithm are stored in the `data` directory.

#### The Map*
`map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id

### All other data the simulator provides, such as observations and controls.

> * Map data provided by 3D Mapping Solutions GmbH.


