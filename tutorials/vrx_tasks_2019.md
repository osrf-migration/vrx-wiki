# VRX Tasks: Examples #

[TOC]

For each of the individual tasks in the 2019 VRX competition (link to document coming soon!) we provide examples of simulation worlds and Gazebo plugins to evaluate and score task performance.  Instructions for running these examples are given below. Please see the Description of Tasks and Technical Guide documents (again, links coming soon) for descriptions of each task, its application interface (API) and the scoring.

## Descriptions of Tasks ##

The individual tasks are described in the competition documents posted on the [Documentation Wiki](https://bitbucket.org/osrf/vrx/wiki/documentation)

## General Instructions: ##

### Initial State ###
After launch, all examples given below should begin with the wamv floating on the water near the
shore, as shown:

![start600.png](https://bitbucket.org/repo/BgXLzgM/images/599749426-start600.png)

Additional course elements will vary from task to task.

### Monitoring Task Status ###

The [vrx_gazebo/Task](https://bitbucket.org/osrf/vrx/src/default/vrx_gazebo/msg/Task.msg) status message includes:

* task state {Initial, Ready, Running, Finished}
* current score
* timing information

Task status is published to `/vrx/task/info` (further details of the API are in the Technical Guide (link coming soon)).  We recommend that you monitor the task status during simulation. One way to do this, for example, is to run:

```
rostopic echo /vrx/task/info 
```
## Individual Tasks ##
The following quick start instructions walk you through the initial process
of launching your environment and subscribing to any available task-specific messages.

### 1. Station-Keeping ###

**Summary**: Navigate to the goal pose and hold station. The best solutions will minimize the difference
between the goal pose and the actual pose of the vehicle over the duration of the task.

1. Start the example: `roslaunch vrx_gazebo station_keeping.launch`
1. Subscribe to the task-specific information provided by the stationkeeping scoring plugin:
    * The station-keeping goal (given as a [geographic_msgs/GeoPoseStamped](http://docs.ros.org/api/geographic_msgs/html/msg/GeoPoseStamped.html)): 
        * `rostopic echo /vrx/station_keeping/goal`
    * The position error values:
        * `rostopic echo /vrx/station_keeping/pose_error`
        * `rostopic echo /vrx/station_keeping/rms_error`
1. For implementation details, see "4.1.1. Task 1: Station-Keeping" in the Competition and Task Descriptions (link coming), or refer to the [stationkeeping scoring plugin](https://bitbucket.org/osrf/vrx/src/default/vrx_gazebo/include/vrx_gazebo/stationkeeping_scoring_plugin.hh)

### 2. Wayfinding ###

**Summary**: Navigate through each of the published waypoints.

1. Start the example: `roslaunch vrx_gazebo wayfinding.launch`
1. Subscribe to the task-specific information provided by the :
    * The list of waypoints (given as a [geographic_msgs/GeoPath](http://docs.ros.org/api/geographic_msgs/html/msg/GeoPath.html)): 
        * `rostopic echo /vrx/wayfinding/waypoints`
    * The position error values:
        * `rostopic echo /vrx/wayfinding/mean_error`
        * `rostopic echo /vrx/wayfinding/min_errors`
1. For implementation details, see "4.1.2. Task 2: Wayfinding" in the Competition and Task Descriptions  (link coming), or refer to the [wayfinding scoring plugin](https://bitbucket.org/osrf/vrx/src/default/vrx_gazebo/include/vrx_gazebo/wayfinding_scoring_plugin.hh)

### 3. Perception ###

1. Start the example: `roslaunch vrx_gazebo perception_task.launch`
1. View the camera feeds from the front of the WAM-V `rosrun rqt_gui rqt_gui --perspective-file ~/vrx_ws/src/vrx/vrx_gazebo/config/front_stereo.perspective` (See a [video](https://vimeo.com/user5784414/review/321818142/3d90192ee0) of buoys appearing in the field of view.) 
1. Publish a landmark identification and localization solution `rostopic pub -r 1 /vrx/perception/landmark geographic_msgs/GeoPoseStamped '{header: {stamp: now, frame_id: "red_mark"}, pose: {position: {latitude: 21.30996, lo
ngitude: -157.8901, altitude: 0.0}}}'`

### 4. Navigation Channel ###

1. Start the example: `roslaunch vrx_gazebo navigation_task.launch verbose:=true`
    * Note that Gazebo messages such as "New gate crossed!" will be printed to the terminal.
1. Use a gamepad to drive the USV through the course (see Driving tutorial) `roslaunch vrx_gazebo usv_joydrive.launch`


### 5. Scan-the-Code and Dock ###

There are two variants of this task.  In the first variant the correct dock is specified via a ROS message.  In the second variant, the correct dock must be deduced from the Scan-the-Code sequence.

#### 5a. Dock Specified via ROS message ####

1. Start the example: `roslaunch vrx_gazebo scan_and_dock_a.launch verbose:=true`
1. Subscribe to the ROS message that specifies the color and shape of corresponding to the placard on the intended dock: `rostopic echo /vrx/scan_dock/placard_symbol`
1. Use a gamepad to dock `roslaunch vrx_gazebo usv_joydrive.launch`

#### 5b. Dock Specified via Scan-the-Code ####

1. Start the example: `roslaunch vrx_gazebo scan_and_dock_b.launch verbose:=true`