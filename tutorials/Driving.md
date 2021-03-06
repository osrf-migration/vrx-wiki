# Overview #

In this tutorial we will demonstrate how to interface with the Gazebo thrust model plugin to move the WAM-V.  For this example we will be using the [propulsion configuration](https://bitbucket.org/osrf/vrx/wiki/tutorials/PropulsionConfiguration) of two, aft, fixed thrusters.  With this configuration, to move the WAM-V we need to have a ROS node that publishes messages on the topics `left_thrust_cmd` and  `right_thrust_cmd` to exercise the left and right thrusters respectively.  The examples below provide possible ways for generating these messages for manual (teleoperated) control.

# Examples #

In the following examples, we'll illustrate how to publish messages to the thrusters simulated by the plugin.  We first start the simulation, as we did in the [Sand Island Basic Tutorial](https://bitbucket.org/osrf/vrx/wiki/tutorials/Sand_Island_Basic), with the command:

```
    $ roslaunch vrx_gazebo sandisland.launch
```

Open a new terminal and source the appropriate `setup.bash` as we describe in the [installation tutorial](https://bitbucket.org/osrf/vrx/wiki/tutorials/SystemSetupInstall). Now, choose one of the following teleoperation options to control the WAM-V. 

## Teleoperation ##
### Teleop option 1: rostopic pub ###

We can use the `rostopic pub` utility to publish individual messages

```
rostopic pub --once /right_thrust_cmd std_msgs/Float32 "data: 1.0"
```

This will publish one message and cause the WAM-V to rotate about the z-axis.  The command will timeout after a pre-determined amount of time (the default is 1.0 s, but can be changed in the [wamv_gazebo_thruster_config.xacro](https://bitbucket.org/osrf/vrx/src/default/wamv_gazebo/urdf/thruster_layouts/wamv_gazebo_thruster_config.xacro) SDF file).


### Teleop option 2: Keyboard ###

To use the keyboard, we use the [teleop_twist_keyboard](http://wiki.ros.org/teleop_twist_keyboard) package, along with a custom [twist2thrust.py]( https://bitbucket.org/osrf/vrx/src/default/vrx_gazebo/nodes/twist2thrust.py) node to convert the Twist messages to two Float32 messages for the left and right thrusters. Forward velocity (twist.linear.x) is mapped to axial thrust (right+left) and rotational velocity (twist.linear.z) is mapped to differential thrust (usvdrive.right-usvdrive.left).

A launch file example is included:

```
roslaunch vrx_gazebo usv_keydrive.launch
```

As of June 30 2019, we have added thruster articulation control functionality to `usv_keydrive.launch`. If thruster articulation is on (look at the [Thruster Articulation](https://bitbucket.org/osrf/vrx/wiki/tutorials/thruster_articulation) page for information about setting this up), you are also able to control the thruster angles relative to the WAM-V.

See the [Keydrive Details](https://bitbucket.org/osrf/vrx/wiki/tutorials/keydrive_details) page for the details of how keyboard teleop is setup for the example WAM-V configurations.

### Teleop option 3: Gamepad ###

To use a gamepad, we use the [joy](http://wiki.ros.org/joy) and [joy_teleop](http://wiki.ros.org/joy_teleop) packages, along with a custom [twist2thrust.py]( https://bitbucket.org/osrf/vrx/src/default/vrx_gazebo/nodes/twist2thrust.py) node to convert Twist messages from the standard teleop to two Float32 messages for the thruster plugin.  The example is...

```
roslaunch vrx_gazebo usv_joydrive.launch
```

For the default configuration, the left stick up/down axis (axis 1) is mapped to the left thruster and the right stick up/down axis (axis 3) is mapped to the right thruster.  Therefore, pushing both sticks forward should cause the WAM-V to drive forward.

As of June 30 2019, we have added thruster articulation control functionality to `usv_joydrive.launch`. If thruster articulation is on (look at the [Thruster Articulation](https://bitbucket.org/osrf/vrx/wiki/tutorials/thruster_articulation) page for information about setting this up), you are also able to control the thruster angles relative to the WAM-V.

If you are using the Logitech F310 with the default configuration, make sure the [Mode light](http://support.logitech.com/en_my/article/21691?product=a0qi00000069ueWAAQ) is unlit.

If you are using a different input device compatible with the joy package, you can edit the [diffdrive.yaml](https://bitbucket.org/osrf/vrx/src/default/vrx_gazebo/config/diffdrive.yaml) configuration file so that the pertinent axis of your device are used.

## Programmatic ##

The [Thruster Articulation wiki](https://bitbucket.org/osrf/vrx/wiki/tutorials/thruster_articulation) illustrates how to use the `rqt_publisher` utility to publish thrust and articulation commands to the WAM-V propulsion system.  The [propulsion system is reconfigurable](https://bitbucket.org/osrf/vrx/wiki/tutorials/PropulsionConfiguration), so the interface may change (topic names, etc.) with different configurations, but the example uses the same `sandisland.launch` starting point.  The example shows how to publish commands to the appropriate ROS topics.

To extend the `rqt_publisher` example to a more general purpose programmatic interface users can take advantage of a number of ways to publish to the ROS API. The [ROS tutorials](http://wiki.ros.org/ROS/Tutorials) provide examples for writing publishers in C++ and Python.  The Mathworks provides examples of publishing from [MATLAB](https://www.mathworks.com/help/robotics/ref/robotics.publisher.html) or [Simulink](https://www.mathworks.com/help/robotics/ref/publish.html).  Tufts University developed a [LabView ROS interface](http://sine.ni.com/nips/cds/view/p/lang/en/nid/213279) that can publish these messages.