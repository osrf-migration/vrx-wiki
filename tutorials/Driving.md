# Overview #

In this tutorial we will demonstrate how to interface with the Gazebo thrust model plugin to move the WAM-V.  

The effect of thrusters on the WAM-V is simulated by the [usv_gazebo_thrust_plugin](https://bitbucket.org/osrf/vmrc/src/default/robotx_gazebo/src/usv_gazebo_thrust_plugin.cc) Gazebo model plugin.  This plugin subscribes to a the custom ROS message, [UsvDrive](https://bitbucket.org/osrf/vmrc/src/default/robotx_gazebo/msg/UsvDrive.msg), on the `cmd_drive` topic. 
 The UsvDrive message defined as having two values to specify the left (port) and right (starboard) normalized thrust commands.  These commands are normalized between -1.0 to 1.0, where 1.0 is maximum forward force and -1.0 is maximum reverse force.  The mapping between the values in the UsvDrive message and the actual force applied to the model are discussed in [Theory of Operation](https://bitbucket.org/osrf/vmrc/wiki/VMRCGazeboPlugins) document.

To move the WAM-V we need to have a ROS node the publishes on UsvDrive messages on the `cmd_drive` topic.  There are a variety of ways we can do this.  The rest of the tutorial will provide examples.

# Examples #

In the examples we'll illustrate publishing UsvDrive messages.  We first start the simulation, as we did in the [Sand Island Basic Tutorial](https://bitbucket.org/osrf/vmrc/wiki/tutorials/Sand_Island_Basic), with the command

```
    $ roslaunch robotx_gazebo sandisland.launch 
```

## rostopic pub ##

We can use the `rostopic pub` utility to publish a message 

```
rostopic pub --once /cmd_drive usv_gazebo_plugins/UsvDrive "left: 0.0 right: 0.0"
```

This will publish one message and cause the WAM-V to rotate about the z-axis.  The command will timeout after a pre-determined amount of time (the default is 1.0 s, but can be changed in the [wamv_gazebo_thrust_plugin.xacro](https://bitbucket.org/osrf/vmrc/src/default/robotx_gazebo/urdf/wamv_gazebo_thrust_plugin.xacro) SDF file).

Similarly we could drive the vessel forward with 

```
rostopic pub --once /cmd_drive robotx_gazebo/UsvDrive '{left: 1.0, right: 1.0}'
```

## Teleop: Keyboard ##

To use the keyboard, we use the [teleop_twist_keyboard](http://wiki.ros.org/teleop_twist_keyboard) package, along with a custom `twist2drive_keyboard.py` node to publish UsvDrive messages on the `cmd_drive` topic.  The `twist2drive_keyboard.py` node converts the Twist message publications from the teleop_twist_keyboard node into UsvDrive messages.  Forward velocity (twist.linear.x) is mapped to axial thrust (usvdrive.right+usvdrive.left) and rotational velocity (twist.linear.z) is mapped to differential thrust (usvdrive.right-usvdrive.left).

A launch file example is included, which can be run with...

```
roslaunch robotx_gazebo usv_keydrive.launch
```

## Teleop: Gamepad ##

To use a gamepad, we use the [joy](http://wiki.ros.org/joy) and [joy_teleop](http://wiki.ros.org/joy_teleop) packages, along with a custom `twist2drive_diff` node to publish UsvDrive messages on the `cmd_drive` topic using a Logitech F310 gamepad.  There is a launch file to add these nodes...

```
roslaunch robotx_gazebo usv_diffdrive.launch
```

For the default configuration, the left stick up/down axis (axis 1) is mapped to the left thruster and the right stick up/down axis (axis 3) is mapped to the right thruster.  Therefore, pushing both sticks forward should cause the WAM-V to drive forward. 

If you are using the Logitech F310 with the default configuration, make sure the [Mode light](http://support.logitech.com/en_my/article/21691?product=a0qi00000069ueWAAQ) is unlit.

If you are using a different input device compatible with the joy package, you can edit the [diffdrive.yaml](https://bitbucket.org/osrf/vmrc/src/default/robotx_gazebo/config/diffdrive.yaml) configuration file so that the pertinent axis of your device are used.