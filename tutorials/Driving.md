# Overview #

In this tutorial we will demonstrate how to interface with the Gazebo thrust model plugin to move the WAM-V.  

The effect of thrusters on the WAM-V is simulated by the [usv_gazebo_thrust_plugin](https://bitbucket.org/osrf/vmrc/src/default/robotx_gazebo/src/usv_gazebo_thrust_plugin.cc) Gazebo model plugin.  This plugin subscribes to a the custom ROS message, [UsvDrive](https://bitbucket.org/osrf/vmrc/src/default/robotx_gazebo/msg/UsvDrive.msg), on the `cmd_drive` topic. 
 The UsvDrive message defined as having two values to specify the left (port) and right (starboard) normalized thrust commands.  These commands are normalized between -1.0 to 1.0, where 1.0 is maximum forward force and 1.0 is maximum reverse force.  The mapping between the values in the UsvDrive message and the actual force applied to the model are discussed in [Theory of Operation](https://bitbucket.org/osrf/vmrc/wiki/VMRCGazeboPlugins) document.

To move the WAM-V we need to have a ROS node the publishes on UsvDrive messages on the `cmd_drive` topic.  There are a variety of ways we can do this.  The rest of the tutorial will provide examples.

# Examples #

In the examples we'll illustrate publishing UsvDrive messages.  We first start the simulation, as we did in the [Sand Island Basic Tutorial](https://bitbucket.org/osrf/vmrc/wiki/tutorials/Sand_Island_Basic), with the command

```
    $ roslaunch robotx_gazebo sandisland.launch 
```

## rostopic pub ##

We can use the `rostopic pub` utility to publish a message 

```
rostopic pub --once /cmd_drive robotx_gazebo/UsvDrive '{left: 0.5, right: -0.5}'
```

This will publish one message and cause the WAM-V to rotate about the z-axis.  The command will timeout after a pre-determined amount of time (the default is 1.0 s, but can be changed in the [wamv_gazebo_thrust_plugin.xacro](https://bitbucket.org/osrf/vmrc/src/default/robotx_gazebo/urdf/wamv_gazebo_thrust_plugin.xacro) SDF file).

Similarly we could drive the vessel forward with 

```
rostopic pub --once /cmd_drive robotx_gazebo/UsvDrive '{left: 1.0, right: 1.0}'
```

## Teleop: Keyboard ##

## Teleop: Gamepad ##




## Custom Node ##