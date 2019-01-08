#CarND-Kidnapped-Vehicle-Project-master
###     Author: Pengmian Yan
### Created on: Jan 02, 2019

[//]: # (Image References)
[image1]: ./data/result.png  "Result passed."
[image2]: ./data/system_overview.png  "system_overview"
[result_gif]: ./data/result.gif



## 1. Project Introduction
A robot car has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

In this project I implemented a 2 dimensional particle filter in C++. The particle filter will be given a map and some initial localization information (analogous to what a GPS would provide). At each time step the filter will also get observation and control data.


## 2. Demo of Result
As a result my particle filter passed the check of error for 100 second.
![alt text][image1]


The small black circles with x are the landmarks on the map. The blue car represents the ground truth data and the big blue circle with a black arrow is the position estimation of my particle filter. The blue lines show which landmarks are associated with observations.Following is a gif animation to show the real-time localizing of the vehicle with my particle filter.
![alt text][result_gif]



## 3. Running the Code
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. ./clean.sh   clean the project
2. ./build.sh   build the project (compiling)
3. ./run.sh     run the project

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)


Here is the main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.

INPUT: values provided by the simulator to the c++ program

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


OUTPUT: values provided by the c++ program to the simulator

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


My particle filter sucessfully localized the car so that the simulator output says:

```
Success! Your particle filter passed!
```

## 4. Implementing the Particle Filter
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

All code are in the `src` directory. The header file `particle_filter.h` declare the the `ParticleFilter` class. The file `particle_filter.cpp` contains the scaffolding of this class and some associated methods. 

The main function is in `src/main.cpp`. This file contains the code that will actually be communicating with th simulator, running the particle filter and calling the associated methods. 

The Particle filter is implemented in following steps:

For onece:

1. Initialization of the particles with GPS data with Gaussian noise;
 
For each time step:

2. Prediction of the next vehicle position using the control data with motion noise;
3. Tranformation of observation from vehicle coordinate to map coordinate;
3. Association between the tranformed observations and landmarks within sensor range (Principle: nearest neighbor);
4. Computing the weights of particles through comparing the positions of observations and associated landmarks (multivariate Gaussian distribution);
5. Resample the particle according to their weights (discrete distribution);
6. send the best particle to the simulator as output.

![alt text][image2]

## 5. Inputs to the Particle Filter
You can find the inputs to the particle filter in the `data` directory.

#### The Map*
`map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id

### All other data the simulator provides, such as observations and controls.

> * Map data provided by 3D Mapping Solutions GmbH.



# CarND-Kidnapped-Vehicle-Project
# CarND-Kidnapped-Vehicle-Project
