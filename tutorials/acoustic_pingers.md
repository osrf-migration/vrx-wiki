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
```

And here's the [message definition](https://bitbucket.org/osrf/vrx/src/acoustic_pinger_plugin/usv_msgs/msg/RangeBearing.msg).

Note that the sensor gives you a value in spherical coordinates: a distance (range) and two angles (bearing and elevation). The value that you'll get from the sensor includes some noise.

# Change the pinger location and visualize sensor measurements in RViz

The topic `/wamv/sensors/pingers/pinger/set_pinger_position` allows you to change the beacon location. You need to publish a Vector3 message. [Here](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Vector3.html)'s the message definition.

For simplicity, we have included a launch file that places the acoustic beacon under one of the entrance gate. Additionally, this launch file publishes RViz markers in the location where the beacon is detected. Let's start by launching `pinger.launch`. Type in a new terminal:

```
roslaunch vrx_gazebo pinger.launch
```

Now, let's start RViz. Type in a new terminal:

```
roslaunch wamv_gazebo localization_example.launch &
roslaunch wamv_gazebo rviz_vrx.launch
```

Click the `Add` button, select the `By topic` tab, and select `/wamv/sensors/pingers/pinger/marker/signal/Marker`, and then, hit OK.

Now, select the WAM-V, hit `t` (translate), and drag and drop the WAM-V next to the entrance gate.

![gazebo_pinger.png](https://bitbucket.org/repo/BgXLzgM/images/4120592019-gazebo_pinger.png)

You should visualize in RViz a blue arrow showing the location of the beacon according to your acoustic sensor.

![rviz_pinger.png](https://bitbucket.org/repo/BgXLzgM/images/2069800313-rviz_pinger.png)