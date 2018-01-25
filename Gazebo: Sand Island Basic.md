Simple base world and vehicle model with environment plugins.

**Beware - This wiki and the relevant source is under development!  Things may change/break at anytime.
**

# Setup

Currently this scenario is only being tested with Ubuntu 16.04, ROS Kinetic and Gazebo 7 (as installed via apt).

The setup below is done in a new catkin workspace (robotx_ws). Because the packages are under development, it is convenient to separate them from any exiting workspaces (e.g., catkin_ws) for testing purposes.

The source repository has a script, with comments, for the setup: *setup_robotx_ws.sh*.  For users familiar with ROS/Gazebo workflow, this should suffice.  When we get closer to a releasable system we'll post more verbose instructions.

# Usage / Example

To run the example, use the provide launch file...

```
roslaunch robotx_gazebo sandisland.launch 
```

You should see a new Gazebo window something like this...
![Screenshot from 2018-01-25 10-59-15_mr.png](https://bitbucket.org/repo/BgXLzgM/images/2097879520-Screenshot%20from%202018-01-25%2010-59-15_mr.png)
You will notice that the USV starts drifting along the x-axis.  That is due to the wind.  More on that later.

If you zoom in your will see the WAM-V model like this...
![Screenshot from 2018-01-25 11-00-30.png.70.png](https://bitbucket.org/repo/BgXLzgM/images/2349056053-Screenshot%20from%202018-01-25%2011-00-30.png.70.png)