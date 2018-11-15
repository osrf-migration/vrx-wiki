# Overview
[RVIZ](http://wiki.ros.org/rviz) is a standard ROS tool for visualizing messages. This tutorial explains how to run rviz to display the WAM-V and sensors that you are simulating with Gazebo.

## Launching Gazebo
If you have not done so already, first run the simulation with the example URDF with sensors or the custom URDF [you created](https://bitbucket.org/osrf/vmrc/wiki/tutorials/AddingSensors):

```
roslaunch vmrc_gazebo sandisland.launch "urdf:=\$(find wamv_gazebo)/urdf/wamv_gazebo_sensors.urdf"
```
Leave this simulation running for the remainder of the tutorial. In the steps below, you will use the robot_state_publisher to make the simulated data available to rviz.

## Publishing a TF tree
RVIZ depends on [TF](http://wiki.ros.org/tf) for understanding where to display everything. 

This is usually accomplished with the robot_state_publisher. This program will publish all the fixed joints in your URDF to the TF tree along with non-fixed joints based on messages published to /JointStates (like the propeller rotations).

There are (at least) two options for doing this:

### Option 1: Using the default URDF

If you are using the example wamv_gazebo_sensors.urdf, you can simply run:
```
$ roslaunch wamv_gazebo localization_example.launch
```
This will run robot_state_publisher along with a localization node which fuses the GPS and IMU into the robot's global pose and publishes it to the odom frame.

### Option 2: Using custom URDF

If you are using a custom URDF without the standard GPS/IMU configuration, you can run the robot_state_publisher alone:
```
$ rosrun robot_state_publisher robot_state_publisher
```

## Running RVIZ
To open RVIZ with a configuration made for the WAM-V, open a third terminal and run:
```
roslaunch wamv_gazebo rviz_example.launch
```

RVIZ should open and display the WAM-V and a camera! Try [driving around](https://bitbucket.org/osrf/vmrc/wiki/tutorials/Driving) to see the robot move both in Gazebo and RVIZ.

![2018-06-28_11-56-12.gif](https://bitbucket.org/repo/BgXLzgM/images/4238727469-2018-06-28_11-56-12.gif)

## Customizing RVIZ
The example launch file above runs rviz with a preconfigured set of topics. You can add, remove, and edit topics and more from within the rviz GUI to better visualize your robot.

## Troubleshooting

### Change Fixed Frame
The example RVIZ config starts with "odom" set as the frame everything is displayed in. If you are using the example URDF with a GPS and IMU and running the localization_example.launch, this frame should exist. However, if you are using a custom URDF/localization which does not publish an odom frame, you can always switch the frame to base_link to display messages relative to the WAM-V:

![rvizswitchframe.png](https://bitbucket.org/repo/BgXLzgM/images/636844775-rvizswitchframe.png)