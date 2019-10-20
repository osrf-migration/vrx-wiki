# Keyboard Teleoperation Details #

The [telop_twist_keyboard](http://wiki.ros.org/teleop_twist_keyboard) is a convenient way to test motion functionality by manually specifying Twist message contents to be published on the `cmd_vel` topic.  The typical use-case for terrestrial robots is the teleoperation provides velocity setpoint commands (forward speed, yaw-rate, etc.) as Twist commands which are inputs to a low-level feedback controller which regulates the motion control.

For the WAM-V use-case we are using this at an even lower level.  We are using the keyboard interface to set the thrust commands provided to the propulsion system.  Following the [Driving](https://bitbucket.org/osrf/vrx/wiki/tutorials/Driving) example should generate a ROS graph that looks like this
![key_drive.png](https://bitbucket.org/repo/BgXLzgM/images/1981347365-key_drive.png) 
The custom `twist2thrust` node translates the published Twist messages as thrust messages for the left and right thrusters.  In effect, this is an open-loop velocity controller where

1) Commanded forward velocity (Twist.linear.x) maps to the propulsion thrust commands
2) Commanded yaw velocity (Twist.angular.z) maps to differential propulsion thrust commands.

In other words:

```
right_thrust_cmd = Twist.linear.x + Twist.angular.z
left_thrust_cmd  = Twist.linear.x - Twist.angular.z
```

We also include the custom `key2thrust_angle` node that takes keyboard commands and publishes [Thruster Articulation](https://bitbucket.org/osrf/vrx/wiki/tutorials/thruster_articulation) commands.  The instructions for using this node are
```
Reading from the keyboard and Publishing Thrust Angles!
---------------------------
Change Thrust Angle clockwise: h
Change Thrust Angle counter-clockwise: ;

r/v : increase/decrease thruster angle change speed by 10%
```

## Propulsion Configurations

The basic setup from the [Driving](https://bitbucket.org/osrf/vrx/wiki/tutorials/Driving) example, and illustrated in the ROS graph above, work for the `H` and `T` [Propulsion Configurations](https://bitbucket.org/osrf/vrx/wiki/tutorials/PropulsionConfiguration).  

For the holonomic 'X' configuration the topic names for the thrusters is a bit different:
```
/wamv/thrusters/right_front_thrust_angle
/wamv/thrusters/right_front_thrust_cmd
/wamv/thrusters/right_rear_thrust_angle
/wamv/thrusters/right_rear_thrust_cmd

```

In this case we need to change the ROS interface between the teleop nodes and the simulated WAMV to include the extra `rear` modifier.  The launch file accepts a command line argument for this case:

```
roslaunch vrx_gazebo sandisland.launch thrust_config:=X
```
```
roslaunch vrx_gazebo usv_keydrive.launch thrust_config:=X
```

![key_drivex.png](https://bitbucket.org/repo/BgXLzgM/images/3768413413-key_drivex.png)