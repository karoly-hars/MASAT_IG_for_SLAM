MASAT intial guess for SLAM
============================================
This repository contains the code for the MASAT algorithm. 

### Guide
Compile:
```sh
./make
```
Run:
```sh
./MASAT 'path_input_file' 'path_to_output_file'
```
The input files must be in .g2o format. There are some examples in the 'input_data' folder.

Once the inital guess is finished, we suggest using the [g2o](https://github.com/RainerKuemmerle/g2o) framework for optimization and visualization.



### Requirements
This repository uses/contains the Eigen library. 
More information about Eigen:
http://eigen.tuxfamily.org/index.php?title=Main_Page

