# Overview #

In this tutorial we will demonstrate how to interface with the Gazebo thrust model plugin to move the WAM-V.  

The effect of thrusters on the WAM-V is simulated by the [usv_gazebo_thrust_plugin](https://bitbucket.org/osrf/vmrc/src/default/usv_gazebo_plugins/src/usv_gazebo_thrust_plugin.cc) Gazebo model plugin.  The plugin is configured using "thruster" tags within the SDF description.  For this example, we are using the [default configuration](https://bitbucket.org/osrf/vmrc/src/default/wamv_gazebo/urdf/dynamics/wamv_gazebo_thrust_plugin.xacro) which defines two thrusters - one at the end of each of the WAM-V hulls.  Within each thruster definition the "cmdTopic" tag specifies the ROS topic to which the plugin is subscribed.  Messages of type [std_msgs/Float32](http://docs.ros.org/kinetic/api/std_msgs/html/msg/Float32.html) are used to describe each thruster input. Each command is a normalized value between -1.0 to 1.0, where 1.0 is maximum forward force and -1.0 is maximum reverse force.  The mapping between this normalized thruster command and the actual force applied to the model is discussed in [Theory of Operation](https://bitbucket.org/osrf/vmrc/wiki/VMRCGazeboPlugins) document.

To move the WAM-V we need to have a ROS node the publishes on messages on the topics `left_thrust_cmd` and  `right_thrust_cmd`  

# Examples #

In the examples we'll illustrate publishing messages the thrusters simulated by the plugin.  We first start the simulation, as we did in the [Sand Island Basic Tutorial](https://bitbucket.org/osrf/vmrc/wiki/tutorials/Sand_Island_Basic), with the command

```
    $ roslaunch robotx_gazebo sandisland.launch 
```

## rostopic pub ##

We can use the `rostopic pub` utility to publish individual messages

```
rostopic pub --once /right_thrust_cmd std_msgs/Float32 "data: 1.0" 
```

This will publish one message and cause the WAM-V to rotate about the z-axis.  The command will timeout after a pre-determined amount of time (the default is 1.0 s, but can be changed in the [wamv_gazebo_thrust_plugin.xacro](https://bitbucket.org/osrf/vmrc/src/default/robotx_gazebo/urdf/wamv_gazebo_thrust_plugin.xacro) SDF file).


## Teleop: Keyboard ##

To use the keyboard, we use the [teleop_twist_keyboard](http://wiki.ros.org/teleop_twist_keyboard) package, along with a custom [twist2drive_keyboard.py](https://bitbucket.org/osrf/vmrc/src/default/robotx_gazebo/nodes/twist2drive_keyboard.py) node to convert the Twist messages to two Float32 messages for the left and right thrusters. Forward velocity (twist.linear.x) is mapped to axial thrust (right+left) and rotational velocity (twist.linear.z) is mapped to differential thrust (usvdrive.right-usvdrive.left).

A launch file example is included...

```
roslaunch robotx_gazebo usv_keydrive.launch
```

## Teleop: Gamepad ##

To use a gamepad, we use the [joy](http://wiki.ros.org/joy) and [joy_teleop](http://wiki.ros.org/joy_teleop) packages, along with a custom [twist2drive_diff](https://bitbucket.org/osrf/vmrc/src/default/robotx_gazebo/nodes/twist2drive_diff.py) node to convert Twist messages from the standard telelop to two Float32 messages for the thruster plugin.  The example is...

```
roslaunch robotx_gazebo usv_joydrive.launch
```

For the default configuration, the left stick up/down axis (axis 1) is mapped to the left thruster and the right stick up/down axis (axis 3) is mapped to the right thruster.  Therefore, pushing both sticks forward should cause the WAM-V to drive forward. 

If you are using the Logitech F310 with the default configuration, make sure the [Mode light](http://support.logitech.com/en_my/article/21691?product=a0qi00000069ueWAAQ) is unlit.

If you are using a different input device compatible with the joy package, you can edit the [diffdrive.yaml](https://bitbucket.org/osrf/vmrc/src/default/robotx_gazebo/config/diffdrive.yaml) configuration file so that the pertinent axis of your device are used.