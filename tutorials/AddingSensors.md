# Overview

This tutorial explains how to create your own robot description file (URDF) for a custom WAM-V with your choice of sensors. These sensors will use gazebo to simulate a camera, lidar, gps, imu, etc and publish the corresponding topics to ROS.


## Prerequisites
This guide assumes you already have installed ROS and the vmrc packages as covered in the [setup instructions](https://bitbucket.org/osrf/vmrc/wiki/tutorials/SystemSetup).


***

## Creating a URDF file
A [URDF](http://wiki.ros.org/urdf) file is a format to describe a robot including joints, sensors, inertial properties, and more. The file is used by gazebo, rviz, and many other ROS packages. Several example URDF files for representing a WAM-V are included in the VMRC packages. 

Let's copy an example locally as a starting point:

*If you have a directory for your project already, copy it there instead*
```
$ mkdir example_vmrc_package
$ cd example_vmrc_package/
$ roscp wamv_gazebo wamv_gazebo_sensors.urdf.xacro my_wamv.urdf.xacro
```
This file contains something like this:
```
cat my_wamv.urdf.xacro
```
```
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro"
       name="WAM-V">
  <!-- Basic frame of WAM-V -->
  <xacro:include filename="$(find wamv_gazebo)/urdf/wamv_gazebo.urdf.xacro" />

  <!-- ADD SENSORS -->
  <!-- Add a front camera -->
  <xacro:wamv_camera name="front_camera"/>
  <!-- Add simulated GPS -->
  <xacro:wamv_gps name="gps_wamv"/>
  <!-- Add Simulated IMU -->
  <xacro:wamv_imu name="imu_wamv"/>
  <!-- Add P3D ground truth -->
  <xacro:wamv_p3d name="p3d_wamv"/>
</robot>
```
Notice that this is an **.xacro** file. If you aren't familiar with xacro files, you should read [this](https://bitbucket.org/osrf/vmrc/wiki/tutorials/Working%20with%20xacro%20files) tutorial first.

This tutorial illustrates adding sensors to the WAM-V simulation

The [Sand Island Basic](https://bitbucket.org/osrf/vmrc/wiki/tutorials/Sand_Island_Basic) tutorial demonstrated the basic functionality of simulating the WAM-V in the RobotX environment.  This base scenario did not include any sensors. 

To customize the scenario you will likely want to add sensors to the simulation.  There are many ways to do this; this tutorial demonstrates doing so by extending the `robot_description` produced by `wamv_robotx.xacro` definition.  As part of this tutorial, a separate git repository ([nps_robotx](https://github.com/bsb808/nps_robotx), for example) is used to accomplish this as a working example. 

The reason you will need a separate repository is that the repository used in the [Sand Island Basic](https://bitbucket.org/osrf/vmrc/wiki/tutorials/Sand_Island_Basic) tutorial is part of the VRMC base, intended to be useful for everyone and not something that you have permissions to change directly (although we encourage pull requests!)  Instead, we will customize the VRMC based by create your own new ROS package.

# Workflow

## Create Your Own ROS Package

For the purposes of this tutorial, you can use the already created [nps_robotx](https://github.com/bsb808/nps_robotx) package.  If you'd like to further customize the simulation, you can fork this repository or create your own.

If you would like to simply follow along, using our pre-baked example, before creating your own package, simply clone the repository into your workspace...

```
cd ~/vmrc_ws/src/
git clone https://github.com/bsb808/nps_robotx.git
```

As usual, you will need to compile and source so that the system is aware of the new package...

```
cd ~/vmrc_ws
catkin_make
source devel/setup.bash 
```

## Create a new robot_description

The WAM-V sensors are simulated using Gazebo model plugins.  These plugins need to be added to your the ```robot_description``` parameter used by Gazebo.  In this example (the nps_robotx package), we created a new xacro file, [nps_robotx.xacro](https://github.com/bsb808/nps_robotx/blob/master/urdf/nps_wamv.xacro) which includes the original WAMV description (from the VRMC repository) and then adds definitions for the sensor plugins (where they are located, what their performance parameters are, etc.)  Each sensor is described in a separate xacro file included from the nps_robotx.xacro definition (gps, imu, camera, etc.)

## Create a new launch file

The launch file is an extension of the sandisland.launch file in the robotx_gazebo package.  By creating your own, you can change the robot_description to our new `nps_robotx.xacro file` (and perhaps add other functionality).

In the `nps_robotx` example repository, there is an `nps_sandisland.launch` file that serves as a working example showing how we can use the original launch file as a starting point and extend it with our own parameters.

## Launch

Now we can try our our new launch file with...

```roslaunch nps_robotx nps_sandisland.launch```

The Gazebo visualization should be the same, but we should now see new ROS topics for our new sensors.  For example, in the rqt_topic window below you can see topics for GPS, IMU, camera and P3D ground truth...
![Screenshot from 2018-02-09 10-12-55.png](https://bitbucket.org/repo/BgXLzgM/images/4243360240-Screenshot%20from%202018-02-09%2010-12-55.png)

## Perspective Camera Example

One way to test our camera setup is to use the `rqt` tool to view the camera feed.  We have a pre-configured configuration file you can use with the following command...
```
rqt --perspective-file ~/vmrc_ws/src/nps_robotx/config/camera.perspective 
```

Now you should be able to see the global view in Gazebo and the USV viewpoint (based on the onboard camera) in rqt...

![Screenshot from 2018-02-26 12-00-19a.png](https://bitbucket.org/repo/BgXLzgM/images/1314066845-Screenshot%20from%202018-02-26%2012-00-19a.png)

We have also posted a video of what you should see if you have the simulation working with our new camera view: https://vimeo.com/257565186