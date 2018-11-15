# Overview

This tutorial explains how to create your own robot description file (URDF) for a custom WAM-V with your choice of sensors. These sensors will use gazebo to simulate a camera, lidar, gps, imu, etc and publish the corresponding topics to ROS.


## Prerequisites
This guide assumes you already have installed ROS and the vmrc packages as covered in the [setup instructions](https://bitbucket.org/osrf/vmrc/wiki/tutorials).


***

## Creating a URDF file
A [URDF](http://wiki.ros.org/urdf) file is a format to describe a robot – including joints, sensors, inertial properties, and more. The file is used by Gazebo, rviz, and many other ROS packages. Several example URDF files for representing a WAM-V are included in the VMRC packages. 

Let's copy an example locally as a starting point:

*If you have a directory for your project already, copy it there instead*
```
$ mkdir example_vmrc_package
$ cd example_vmrc_package/
$ roscp wamv_gazebo wamv_gazebo_sensors.urdf.xacro my_wamv.urdf.xacro
```

Notice that the file you copied is a **.xacro** file. If you aren't familiar with xacro files, you should read [this](https://bitbucket.org/osrf/vmrc/wiki/tutorials/Working%20with%20xacro%20files) tutorial first.

The [original xacro file](https://bitbucket.org/osrf/vmrc/src/default/wamv_gazebo/urdf/wamv_gazebo_sensors.urdf.xacro) defines the base WAM-V, the propulsion configuration, and then adds some example sensors using xacro tags.


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
* A Python expression ```${radians(15)}``` was used to convert 15 degrees to radians



### Running vmrc_gazebo with a custom WAM-V URDF
Now that you have a custom URDF modeling your WAM-V, let's run the simulation!

First, generate the compiled XML from the xacro file using the command below (or [another method](https://bitbucket.org/osrf/vmrc/wiki/tutorials/Working%20with%20xacro%20files)):
```
$ rosrun xacro xacro --inorder my_wamv.urdf.xacro > my_wamv.urdf
```
Next, run the simulation with a custom urdf argument:
```
$ roslaunch vmrc_gazebo sandisland.launch urdf:=`pwd`/my_wamv.urdf
```
You can open rviz/rqt to see your new sensors (for help on how to do this, see the [RVIZ tutorial](https://bitbucket.org/osrf/vmrc/wiki/tutorials/Visualizing%20with%20RVIZ)):

![newcameras.png](https://bitbucket.org/repo/BgXLzgM/images/2924402190-newcameras.png)

## Changing the Propulsion Configuration

In this tutorial you explicitly specify the urdf argument as input to the [sandisland.launch](https://bitbucket.org/osrf/vmrc/src/default/vmrc_gazebo/launch/sandisland.launch) file.  This overrides the method used in the [Propulsion Configuration Tutorial](https://bitbucket.org/osrf/vmrc/wiki/tutorials/PropulsionConfiguration) to specify the thruster layout.  

The original [wamv_gazebo_sensors.urdf.xacro](https://bitbucket.org/osrf/vmrc/src/default/wamv_gazebo/urdf/wamv_gazebo_sensors.urdf.xacro) file includes examples for how to specify the 'T' and 'X' propulsion configuration within your new custom URDF file.

For example, if you want to change from the two-aft thruster configuration to the 'T' configuration, substitute the line in `my_wamv.urdf.xacro` from 

```
<xacro:wamv_gazebo thruster_layout="$(find wamv_gazebo)/urdf/thruster_layouts/wamv_aft_thrusters.xacro"/>
```

to 

```
<xacro:wamv_gazebo thruster_layout="$(find wamv_gazebo)/urdf/thruster_layouts/wamv_t_thrusters.xacro"/>
```

Then repeat the execution steps:
```
$ rosrun xacro xacro --inorder my_wamv.urdf.xacro > my_wamv.urdf
```

```
$ roslaunch vmrc_gazebo sandisland.launch urdf:=`pwd`/my_wamv.urdf
```


## Next steps:

 * To visualize sensors in ROS, check out the [RVIZ tutorial](https://bitbucket.org/osrf/vmrc/wiki/tutorials/Visualizing%20with%20RVIZ)