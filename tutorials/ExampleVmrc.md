# VMRC Example #

This example illustrates a specific configuration similar to what some of the RobotX teams used in 2018.  It includes:

* Fixed stern thrusters and a single lateral thuster - the ['T' propulsion configuration](https://bitbucket.org/osrf/vmrc/wiki/tutorials/PropulsionConfiguration)
* A standard set of onboard sensors
  * Two forward facing cameras (stereo)
  * One lateral camera pointing right
  * GPS
  * IMU
  * 3D LiDAR

## Run the example

```
roslaunch robotx_gazebo vmrc.launch 
```

## Visualize the sensor data

```
roslaunch wamv_gazebo rviz_vmrc.launch 
```