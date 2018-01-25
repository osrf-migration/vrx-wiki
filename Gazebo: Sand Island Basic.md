Simple base world and vehicle model with environment plugins.

This is under development!


# Setup

Currently this scenario is only being tested with Ubuntu 16.04, ROS Kinetic and Gazebo 7 (as installed via apt).

The setup below is done in a new catkin workspace (robotx_ws). Because the packages are under development, it is convenient to separate them from any exiting workspaces (e.g., catkin_ws) for testing purposes.

The source repository has a script, with comments, for the setup: *setup_robotx_ws.sh*.  For users familiar with ROS/Gazebo workflow, this should suffice.  When we get closer to a releasable system we'll post more verbose instructions.

# Usage

To run the example, use the provide launch file...

```
roslaunch robotx_gazebo sandisland.launch 
```