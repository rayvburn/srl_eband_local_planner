srl_eband_local_planner
===================

The [srl_eband_local_planner] is
a [ROS] (http://wiki.ros.org) package, implementing a plugin to the
[base_local_planner](http://wiki.ros.org/base_local_planner) for the
[move_base](http://wiki.ros.org/move_base) 2D navigation system.

It implements the Elastic Band method on the SE2 manifold.
include spline interpolation in trajectory controller.

Differently from the original package this version uses a path follower based on input/output linearization of a unicycle model.

## Requirements
* ROS (including visualization rools -> rviz), tested on Indigo and Hydro
* ros-hydro-navigation or ros-indigo-navigation
* Eigen3
* Boost >= 1.46
* C++11 compiler
* spencer_tracking_msgs
* spencer_control_msgs

## Installation

Clone the package into you catkin workspace
- `cd [workspace]/src`
- `git clone https://github.com/srl-freiburg/srl_eband_local_planner.git`
- `cd ../`
- `catkin_make` or `catkin build`
