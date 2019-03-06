# VRX Tasks: Examples #

[TOC]

The individual tasks for the 2019 VRX competition (link to document coming soon!) are supported by example simulation worlds and Gazebo plugins to evaluate and score task performance.  Below are examples that illustrate the implementation of the these tasks.  For descriptions of the task, the application interface (API) for each task and the scoring, please see the Description of Tasks and Technical Guide documents (again, links coming soon).

## General Instructions ##

### Monitoring Task Status ###

The [vrx_gazebo/Task](https://bitbucket.org/osrf/vrx/src/default/vrx_gazebo/msg/Task.msg) status message includes, task state {Initial, Ready, Running, Finished}, current score and timing information (details of the API are in the Technical Guide (link coming soon)).  Recommended that you monitor the status during simulation, e.g.,

```
rostopic echo /vrx/task/info 
```

## 1. Station-Keeping ##

1. Start the Gazebo example: `roslaunch vrx_gazebo station_keeping.launch`
1. Subscribe to the task-specific information provided by the [stationkeeping scoring plugin](https://bitbucket.org/osrf/vrx/src/default/vrx_gazebo/include/vrx_gazebo/stationkeeping_scoring_plugin.hh):
    * The station-keeping goal: ` rostopic echo /vrx/station_keeping/goal`
    * The position error values:
        * `rostopic echo /vrx/station_keeping/pose_error`
        * `rostopic echo /vrx/station_keeping/rms_error`

## 2. Wayfinding ##

1. Start the Gazebo example: ` roslaunch vrx_gazebo wayfinding.launch`
1. Subscribe to the task-specific information provided by the [wayfinding scoring plugin](https://bitbucket.org/osrf/vrx/src/default/vrx_gazebo/include/vrx_gazebo/wayfinding_scoring_plugin.hh):
    * The waypoint goals as a [geographic_msgs/GeoPath](http://docs.ros.org/api/geographic_msgs/html/msg/GeoPath.html): ` rostopic info /vrx/wayfinding/waypoints`
    * The position error values:
        * `rostopic echo /vrx/wayfinding/mean_error`
        * `rostopic echo /vrx/wayfinding/min_errors`

## 3. Perception ##

1. Start the Gazebo example: `roslaunch vrx_gazebo perception_task.launch`
1. View the camera feeds from the front of the WAM-V `rosrun rqt_gui rqt_gui --perspective-file ~/vrx_ws/src/vrx/vrx_gazebo/config/front_stereo.perspective` (See a [video](https://vimeo.com/user5784414/review/321818142/3d90192ee0) of buoys appearing in the field of view.) 
1. Publish a landmark identification and localization solution `rostopic pub -r 1 /vrx/perception/landmark geographic_msgs/GeoPoseStamped '{head
er: {stamp: now, frame_id: "red_mark"}, pose: {position: {latitude: 21.30996, lo
ngitude: -157.8901, altitude: 0.0}}}'`

## 4. Navigation Channel ##

1. Start the Gazebo example: `roslaunch vrx_gazebo navigation_task.launch verbose:=true`
    * Note that Gazebo messages such as "New gate crossed!" will be printed to the terminal.
1. Use a gamepad to drive the USV through the course (see Driving tutorial) `roslaunch vrx_gazebo usv_joydrive.launch`


## 5. Scan-the-Code and Dock ##

There are two variants of this task.  In the first variant the correct dock is specified via a ROS message.  In the second variant, the correct dock must be deduced from the Scan-the-Code sequence.

### 5a. Dock specified via ROS message ###

1. Start the Gazebo example: `roslaunch vrx_gazebo scan_and_dock_a.launch verbose:=true`
1. Subscribe to the ROS message that specifies the color and shape of corresponding to the placard on the intended dock: `rostopic echo /vrx/scan_dock/placard_symbol`
1. Use a gamepad to dock `roslaunch vrx_gazebo usv_joydrive.launch`

### 5b. Dock specified via Scan-the-Code ###

1. Start the Gazebo example: `roslaunch vrx_gazebo scan_and_dock_b.launch verbose:=true`