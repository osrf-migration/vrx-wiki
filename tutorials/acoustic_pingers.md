# Overview #

Some of the RobotX Challenge tasks contain an underwater acoustic beacon. As an example, in the "entrance and exit gates" task, teams should detect the active beacon and transit through the gate in which the active beacon is located.

# Read the acoustic beacon

The VRX WAM-V is equipped with a sensor that detects the underwater acoustic beacon. As usual, a ROS topic is periodically published with the data provided by the sensor in the topic `/wamv/sensors/pingers/pinger/range_bearing`. Let's see an example:

* Launch the VRX simulation:

```
roslaunch vrx_gazebo vrx.launch
```

* Show the data published by the acoustic sensor:

```
rostopic echo /wamv/sensors/pingers/pinger/range_bearing
```

You should see an output similar to this one:

```
header: 
  seq: 30
  stamp: 
    secs: 31
    nsecs:  56000000
  frame_id: "wamv/pinger"
range: 189.372650146
bearing: 0.219589501619
elevation: -0.0155547577888
---

And here's the [message definition](https://bitbucket.org/osrf/vrx/src/acoustic_pinger_plugin/usv_msgs/msg/RangeBearing.msg).

Note that the sensor gives you a value in spherical coordinates: a distance (range) and two angles (bearing and elevation). The value that you'll get from the sensor includes some noise.
```