
# Visual-Inertial Odometry

This repository contains our implementation of a Visual-Inertial Odometry (VIO) system using the Multi-State Constraint Kalman Filter (MSCKF) algorithm. The system processes stereo images and IMU measurements to estimate the trajectory of a moving platform.

## Project Overview

Visual-Inertial Odometry is a crucial technology for estimating the motion of robots and autonomous vehicles by fusing data from cameras and inertial measurement units (IMUs). Our implementation follows the algorithm described in the original MSCKF paper by Mourikis and Roumeliotis, with modifications to handle stereo camera setups.

## Requirements
* Python 3.6+
* numpy
* scipy
* cv2
* [pangolin](https://github.com/uoip/pangolin) (optional, for trajectory/poses visualization)

## Dataset
* [EuRoC MAV](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets): visual-inertial datasets collected on-board a MAV. The datasets contain stereo images, synchronized IMU measurements, and ground-truth.  
This project implements data loader and data publisher for EuRoC MAV dataset.

## Implementation Details

### Core Components

- **IMU State Initialization**: Estimates initial gravity direction and gyroscope bias
- **IMU State Propagation**: Implements 4th order Runge-Kutta numerical integration
- **State Augmentation**: Adds new camera states to the filter state vector
- **Feature Processing**: Detects and tracks features across frames
- **Measurement Update**: Incorporates visual measurements into the filter


### Execution

## Run  
`python vio.py --view --path path/to/your/EuRoC_MAV_dataset/MH_01_easy`  
or    
`python vio.py --path path/to/your/EuRoC_MAV_dataset/MH_01_easy` (no visualization)  


## File Structure

- `vio.py`: Main entry point for the VIO system
- `msckf.py`: Implementation of the MSCKF algorithm
- `image.py`: Image processing and feature tracking
- `config.py`: Configuration parameters
- `viewer.py`: Visualization using Pangolin
- `dataset.py`: Dataset loading and processing

## License and References
Follow [license of msckf_vio](https://github.com/KumarRobotics/msckf_vio/blob/master/LICENSE.txt). Code is adapted from [this implementation](https://github.com/uoip/stereo_msckf).
