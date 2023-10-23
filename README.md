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

## Limitations

This fork provides compatibility of the original planner with the ROS `melodic` (not tested with `noetic`). However, it only upgrades code sections that were completely outdated and threw errors during a compilation. The planner still needs a major refactor, i.a., code sections using legacy `tf` conversions are left as they were.

The planner was not tested with the provided `.launch` files after modifications (custom ones were prepared). Also, note that some `SPENCER` project packages are missing on GitHub.
It is crucial to set the parameters of classes from `behavior_layers` package as the default ones will immediately produce assertion/topic naming errors (ill-formed component).

This planner currently works only with the [`SPENCER`](https://github.com/spencer-project/spencer_people_tracking) perception stack.

Switching to the standard [`people_msgs`](http://wiki.ros.org/people_msgs) (possibly with the handy [converting tools](https://github.com/rayvburn/people_msgs_utils)) would allow testing with different perception systems.
