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

Let's look at the contents within the **<robot>** tag, which describes the robot. The first line includes wamv_gazebo.urdf.xacro. This adds the basic WAM-V mesh and joints along with the plugins for dynamics. You will likely want to keep this in, unless you are using a different model or dynamics simulation.

After that several macros are added for a GPS, IMU, and ground truth pose. These macros are found in [wamv_gazebo](https://bitbucket.org/osrf/vmrc/src/default/wamv_gazebo/urdf/) for common sensors. You can of course create your own, following those as examples.

Let's add a stereo camera pair to the robot. Add the following lines after the other sensors:
```
  <xacro:property name="stereo_x" value="1.0" />
  <xacro:wamv_camera name="stereo_left" x="${stereo_x}" y="0.3" z="1.5" P="${radians(15)}" />
  <xacro:wamv_camera name="stereo_right" x="${stereo_x}" y="-0.3" z="1.5" P="${radians(15)}" />
```
A couple things to notice about this:

* A common property "stereo_x" is used so the value is not copied in multiple places
* The x,y,z and P (pitch) set where the cameras are located relative to the WAM-V base link
* A python expression ```${radians(15)}``` was used to convert 15 degrees to radians



### Running robotx_gazebo with a custom WAM-V URDF
Now that you have a custom URDF modeling your WAM-V, let's run the simulation!

First, generate the compiled XML from the xacro file using this or [another method](https://bitbucket.org/osrf/vmrc/wiki/tutorials/Working%20with%20xacro%20files):
```
$ rosrun xacro xacro --inorder my_wamv.urdf.xacro > my_wamv.urdf
```
Next, run the simulation with a custom urdf argument:
```
$ roslaunch robotx_gazebo sandisland.launch urdf:=`pwd`/my_wamv.urdf
```
You can open rviz/rqt to see your new sensors:

![newcameras.png](https://bitbucket.org/repo/BgXLzgM/images/2924402190-newcameras.png)

### Next steps:
To visualize sensors in ROS, check out the [RVIZ tutorial](https://bitbucket.org/osrf/vmrc/wiki/tutorials/Visualizing%20with%20RVIZ)