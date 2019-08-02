# Overview
[RViz](http://wiki.ros.org/rviz) is a standard ROS tool for visualizing messages. This tutorial explains how to run RViz to display the WAM-V and sensors that you are simulating with Gazebo.

## Launching Gazebo
If you have not done so already, first run the simulation with a front camera and the IMU:

```
roslaunch vrx_gazebo vrx.launch camera_enabled:=true gps_enabled:=true imu_enabled:=true
```
Leave this simulation running for the remainder of the tutorial. In the steps below, you will use the robot_state_publisher to make the simulated data available to RViz.

## Publishing a TF tree
RViz depends on [TF](http://wiki.ros.org/tf) for understanding where to display everything.

This is usually accomplished with the robot_state_publisher. This program will publish all the fixed joints in your URDF to the TF tree along with non-fixed joints based on messages published to /JointStates (like the propeller rotations).

There are (at least) two options for doing this:

### Option 1: Using the default URDF

If you are using the example wamv_gazebo.urdf, you can simply run:
```
$ roslaunch wamv_gazebo localization_example.launch
```
This will run robot_state_publisher along with a localization node which fuses the GPS and IMU into the robot's global pose and publishes it to the odom frame.

### Option 2: Using custom URDF

If you are using a custom URDF without the standard GPS/IMU configuration, you can run the robot_state_publisher alone:
```
$ rosrun robot_state_publisher robot_state_publisher
```

## Running RViz
To open RViz with a configuration made for the WAM-V, open a third terminal and run:
```
roslaunch wamv_gazebo rviz_vrx.launch
```

RViz should open and display the WAM-V and a camera! Try [driving around](https://bitbucket.org/osrf/vrx/wiki/tutorials/Driving) to see the robot move both in Gazebo and RViz.

![2018-06-28_11-56-12.gif](https://bitbucket.org/repo/BgXLzgM/images/4238727469-2018-06-28_11-56-12.gif)

## Customizing RViz
The example launch file above runs RViz with a preconfigured set of topics. You can add, remove, and edit topics and more from within the RViz GUI to better visualize your robot.

## Troubleshooting

### Change Fixed Frame
The example RViz config starts with "odom" set as the frame everything is displayed in. If you are using the example URDF with a GPS and IMU and running the localization_example.launch, this frame should exist. However, if you are using a custom URDF/localization which does not publish an odom frame, you can always switch the frame to base_link to display messages relative to the WAM-V:

![rvizswitchframe.png](https://bitbucket.org/repo/BgXLzgM/images/636844775-rvizswitchframe.png)