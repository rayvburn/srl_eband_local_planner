srl_eband_local_planner
===================

The srl_eband_local_planner is
a [ROS] (http://wiki.ros.org) package, that implements a plugin to the
[base_local_planner](http://wiki.ros.org/base_local_planner) for the
[move_base](http://wiki.ros.org/move_base) 2D navigation system.
It is based on the original implementation by Christian Connette and Bhaskara Marathi.
This local planner has been adapted primarily for differential drive robots,
but still supports the original holonomic drive controls.

## Requirements
* ROS (including visualization rools -> rviz), tested on Indigo and Hydro
* ros-hydro-navigation or ros-indigo-navigation
* Eigen3
* Boost >= 1.46
* C++11 compiler
* spencer_tracking_msgs

## Installation

Clone the package into you catkin workspace
- `cd [workspace]/src`
- `git clone https://github.com/palmieri/srl_eband_local_planner.git`
- `cd ../`
- `catkin_make` or `catkin build`
