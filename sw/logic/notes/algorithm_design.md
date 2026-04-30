# Overview

In the simple and static environment, we can split the problem into 4 steps:

1. Navigate through the zig-zag
2. Locate the bear
3. Pick up the bear
4. Return through the zig-zag to the starting position


## State estimation

The robot uses odometry and LIDAR data for localization. Odometry provides an estimate of the robot's instant shift of position and orientation, while LIDAR is used for correction of odometry errors and for bear detection. We use a simple particle filter for state estimation. State is represented as a tuple of (x, y, theta).

**Motion model**: We use a standard differential drive model. Noise is added to the final coordinates and orientation to account for uncertainty in movement. No noise is added to the odometry readings.

**Sensor model**: As using all measurements from the LIDAR for each particle is computationally unfeasible, we randomly sample ~2% of the measurements for each particle. For each sampled measurement, we calculate the expected distance in given direction and compare it to the actual measurement. The likelihood of the particle is updated based on the error from the expected distance and is modeled as a Gaussian distribution with added baseline to model general uncertainty.


## Path planning

As the map is static and relatively simple, we use checkpoint-based navigation for navigating the zig-zag path.

The robot moves from one checkpoint to the next using a simple proportional controller for steering. Once the robot reaches the end of the path, it can search for the bear.
