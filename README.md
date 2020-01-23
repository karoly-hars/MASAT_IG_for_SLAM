MASAT intial guess for SLAM
============================================
This repository contains the code for the *MASAT algorithm*: a robust and efficent approach for creating inital estimates prior to pose-graph optimization in off-line SLAM.

### The paper describing the approach:
[Károly Harsányi, Attila Kiss, Tamás Szirányi, András Majdik: MASAT: A fast and robust algorithm for pose-graph initialization](https://www.sciencedirect.com/science/article/pii/S0167865519303241)

### Requirements
- Eigen (http://eigen.tuxfamily.org)

### Guide
Compile:
```sh
./make
```
Run:
```sh
./MASAT <PATH_TO_INPUT_FILE> <PATH_TO_OUTPUT_FILE>
```
The input files must be in .g2o format. There are some examples in the ```./input_data``` folder.

Once the inital guess is finished, we suggest using the [g2o](https://github.com/RainerKuemmerle/g2o) framework for optimization and visualization.
