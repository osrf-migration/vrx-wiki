# Overview

This tutorial illustrates adding sensors to the WAM-V simulation

The [Sand Island Basic](https://bitbucket.org/osrf/vmrc/wiki/tutorials/Sand_Island_Basic) tutorial demonstrated the basic functionality of simulating the WAM-V in the RobotX environment.  This base scenario did not include any sensors. 

To customize the scenario you will likely want to add sensors to the simulation.  There are many ways to do this; this tutorial demonstrates doing so by extending the `robot_description` produced by `wamv_robotx.xacro` definition.  As part of this tutorial, a separate GIT repository ([nps_robotx](https://github.com/bsb808/nps_robotx) is used to accomplish this as a working example. Ideally you would not need to edit the repositories used in the [Sand Island Basic](https://bitbucket.org/osrf/vmrc/wiki/tutorials/Sand_Island_Basic) tutorial, but would create your own new ROS package (by forking or simply copying) this example.

# Workflow

## Create Your Own ROS Package

Create a new package - either a blank package or fork/copy the [nps_robotx](https://github.com/bsb808/nps_robotx) package.


## Create a new robot_description

The WAM-V sensors are simulated using Gazebo model plugins.  These plugins need to be added to your the robot_description used by Gazebo.  In this example, we created a new xacro file, [nps_robotx.xacro](https://github.com/bsb808/nps_robotx/blob/master/urdf/nps_wamv.xacro) which includes the WAMV description and adds definitions for the sensor plugins (where they are located, what their performance parameters are, etc.)  Each sensor is described in a separate xacro file included from the nps_robotx.xacro definition (gps, imu, camera, etc.)

## Create a new launch file

The launch file is an extension of the sandisland.launch file in the robotx_gazebo package.  By creating your own, you can change the robot_description to our new nps_robotx.xacro file (and perhaps add other functionality).

## Launch

Now we can try our our new launch file with `roslaunch nps_robotx nps_sandisland.launch`.  The Gazebo visualization should be the same, but we should now see new ROS topics for our new sensors.  For example, in the rqt_topic window below you can see topics for GPS, IMU, camera and P3D ground truth...
![Screenshot from 2018-02-09 10-12-55.png](https://bitbucket.org/repo/BgXLzgM/images/4243360240-Screenshot%20from%202018-02-09%2010-12-55.png)