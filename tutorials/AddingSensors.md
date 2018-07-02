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


### Running robotx_gazebo with a custom WAM-V URDF
Now that you have a custom URDF modeling your WAM-V, let's run the simulation 

### Next steps:
To visualize sensors in ROS, check out the [RVIZ tutorial](https://bitbucket.org/osrf/vmrc/wiki/tutorials/Visualizing%20with%20RVIZ)