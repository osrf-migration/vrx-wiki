The effect of thrusters on the WAM-V is simulated by the [usv_gazebo_thrust_plugin](https://bitbucket.org/osrf/vmrc/src/default/usv_gazebo_plugins/src/usv_gazebo_thrust_plugin.cc) Gazebo model plugin.  The plugin is configured using "thruster" tags within the SDF description.  For this example, we are using the default configuration - the fixed aft thrusters.  The propulsion configuration takes place in the [wamv_aft_thrusters.xacro](https://bitbucket.org/osrf/vmrc/src/default/wamv_gazebo/urdf/thruster_layouts/wamv_aft_thrusters.xacro) definition file which makes use of the general purpose xacro macro defined in [wamv_gazebo_thruster_config.xacro](https://bitbucket.org/osrf/vmrc/src/default/wamv_gazebo/urdf/thruster_layouts/wamv_gazebo_thruster_config.xacro).  Within each thruster definition the "cmdTopic" tag specifies the ROS topic to which the plugin is subscribed.  Messages of type [std_msgs/Float32](http://docs.ros.org/kinetic/api/std_msgs/html/msg/Float32.html) are used to describe each thruster input. Each command is a normalized value between -1.0 to 1.0, where 1.0 is maximum forward force and -1.0 is maximum reverse force.  The mapping between this normalized thruster command and the actual force applied to the model is discussed in [Theory of Operation](https://bitbucket.org/osrf/vmrc/wiki/VMRCGazeboPlugins) document.

# Example Configurations 

The image below illustrates a few possible thruster configurations.

![Propulsion Options.png](https://bitbucket.org/repo/BgXLzgM/images/2101300599-Propulsion%20Options.png)
 
The topics below illustrate how to run three of the four configurations.  The "Differential Thrust. Variable Angle" configuration is not currently supported.

## "H": Differential Thrust with Two, Fixed Stern Thrusters #

The default configuration is the "H" configuration with two, fixed stern thrusters. See the [Driving](https://bitbucket.org/osrf/vmrc/wiki/tutorials/Driving) tutorial.
 
If you run the example simulation without any command line options

```
roslaunch vmrc_gazebo sandisland.launch 
```

The thruster configuration should like similar to the image below.

![wamv_full_H.png](https://bitbucket.org/repo/BgXLzgM/images/3341119966-wamv_full_H.png)


# "T": Differential Thrust with An Additional Lateral/Bow Thruster #

To add a single lateral thruster requires the following changes:

  1. Define new URDF/xacro definition of the WAM-V that includes both the base vehicle (without thrusters) and then adds thrusters to the vessel.  For this example, the new definition is [wamv_gazebo_t.urdf.xacro](https://bitbucket.org/osrf/vmrc/src/default/wamv_gazebo/urdf/wamv_gazebo_t.urdf.xacro).  
  1. This WAM-V definition file includes a propulsion layout definition.  For this example, the new thruster layout is defined in [wamv_t_thruster.xacro](https://bitbucket.org/osrf/vmrc/src/default/wamv_gazebo/urdf/thruster_layouts/wamv_t_thrusters.xacro)

This example is already available in the repository and can be specified using the `thrust_config` command line option of the launch file ...

```
roslaunch vmrc_gazebo sandisland.launch thrust_config:=T
```

To generate a WAM-V propulsion/thruster configuration that looks like the image below.

![wamv_full_t.png](https://bitbucket.org/repo/BgXLzgM/images/3753451461-wamv_full_t.png)



# "X": Holonomic Thruster Configuration with Four Fixed Thrusters

Using the same process describe above, we can add a new set of URDF descriptions (visual and plugin) to define the holonomic configuration.  As an example, we can call this new setup by specifying the "X" thrust configuration...


```
roslaunch vmrc_gazebo sandisland.launch thrust_config:=X
```

![wamv_full_x.png](https://bitbucket.org/repo/BgXLzgM/images/1776480031-wamv_full_x.png)


![wamv_full_x2.png](https://bitbucket.org/repo/BgXLzgM/images/2544748307-wamv_full_x2.png)