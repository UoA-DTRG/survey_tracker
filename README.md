# Survey Tracker
This repository contains code for trajectory tracking on PX4/ROS using Model Predictive Control

## Prerequisites
While this repository should remain compatible with other versions of ROS1/PX4, it has been developed and tested with the following:

 - Ubuntu 20.04.3
 - ROS1 Noetic
 - PX4 SITL v1.12.3

## Installation
You may skip the initial parts of this section if you have already installed our simulator. While this package can be run without simulation, we highly recommend trying it out in simulation first.

This (and the rest of the survey series packages) is designed to be used with the catkin build system. You will need to install some extra dependencies on top of a clean ROS install

```
sudo apt install python3-catkin-pkg python3-catkin-tools
```

Since MAVROS and geographiclib is not installed as part of a base ROS install

```
sudo apt install ros-noetic-mavros ros-noetic-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod +x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh
rm install_geographiclib_datasets.sh
```

If you haven't yet done so, setup a new catkin workspace

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin confg -DCMAKE_BUILD_TYPE=Release
```

Clone this repository 

```
cd ~/catkin_ws/src
git clone git@github.com:tlin442/survey_tracker.git
```

Install build dependencies
```
sudo apt install libeigen3-dev liblapacke-dev
```

Build the packages
```
catkin build
```

## Running the trajectory tracker
Assuming you have our simulation package already running, simply open a new terminal and run
```
source devel/setup.bash && roslaunch survey_tracker path_follower_sim.launch
```

We have provided a script which publishes a lemniscate trajectory. Run this command in the terminal after taking off and switching to offboard
```
source devel/setup.bash && rosrun survey_tracker test_fig8
```

The UAV should automatically follow a lemniscate trajectory. You may need to run the second command more than once to ensure the starting location is correct, otherwise the trajectory tracker will provide aggressive inputs to converge on the position setpoint.

## Acknowledgements
If you use this work in your academic work, please cite our paper
```
[1] Lin, T. and Stol, K.A., Fast Trajectory Tracking of Multi-Rotor UAVs using First-Order Model Predictive Control, 2021 Australian Conference on Robotics and Automation (ACRA), Melbourne, Australia
```

We use [ACADO and the QPOASES solver](https://acado.github.io/) to solve the MPC problem, which are licensed under GPLv2