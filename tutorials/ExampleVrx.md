# VRX Example #

This example illustrates a specific configuration similar to what some of the RobotX teams used in 2018.  It includes:

* Fixed stern thrusters and a single lateral thuster - the ['T' propulsion configuration](https://bitbucket.org/osrf/vrx/wiki/tutorials/PropulsionConfiguration)
* A standard set of onboard sensors
  * Two forward facing cameras (stereo)
  * One starboard facing camera
  * GPS
  * IMU
  * 3D LiDAR

## Simulation

Start Gazebo and spawn WAM-V with propulsion and sensor configuration.

```
roslaunch vrx_gazebo vrx.launch
```
![vrx.jpg](https://bitbucket.org/repo/BgXLzgM/images/3012785430-vrx.jpg)

## Robot Localization

Visualization (and a number of other capabilities) benefit from having a fixed local frame.  The GPS sensor provides localization in a fixed frame, but having a local "odom" frame helps us avoid having to visualize the entire globe!

We can use the [robot_localization](http://wiki.ros.org/robot_localization) package to fuse the GPS and IMU data to generate a position solution, and TF transforms, within the local odom frame, where the location of the local frame is specified by the user. To try this out, start Gazebo with the command above, then open a new terminal or tab and run:

```
roslaunch wamv_gazebo localization_example.launch
```

## Visualization

After executing both commands above, open a third terminal and start RViz with example configuration file to read URDF and sensors. Follow [these tutorials](http://wiki.ros.org/rviz/Tutorials) for an introduction to RViz.

```
roslaunch wamv_gazebo rviz_vrx.launch
```

![rviz_vrx.png](https://bitbucket.org/repo/BgXLzgM/images/1103626992-rviz_vrx.png)