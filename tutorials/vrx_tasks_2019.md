# VRX Tasks: Examples #

[TOC]

For each of the individual tasks in the 2019 VRX competition we provide examples of simulation worlds and Gazebo plugins to evaluate and score task performance.  Instructions for running these examples are given below. Please see the [Description of Tasks](https://bitbucket.org/osrf/vrx/downloads/VRX%202019%20Task%20Descriptions_v1.1.pdf) and [Technical Guide](https://bitbucket.org/osrf/vrx/downloads/VRX%202019%20Technical%20Guide_v1.1.pdf) documents for descriptions of each task, its application interface (API) and the scoring.

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

Task status is published to `/vrx/task/info` (further details of the API are in the [Technical Guide](https://bitbucket.org/osrf/vrx/downloads/VRX%202019%20Technical%20Guide_v1.1.pdf)).  We recommend that you monitor the task status during simulation. One way to do this, for example, is to run:

```
rostopic echo /vrx/task/info
```

## Driving instructions

In preparation for developing an automated solution, we recommend using a gamepad or keyboard to drive the USV through the course (see [Driving tutorial](https://bitbucket.org/osrf/vrx/wiki/tutorials/Driving)):

  * Gamepad: `roslaunch vrx_gazebo usv_joydrive.launch`
  * Keyboard: `roslaunch vrx_gazebo usv_keydrive.launch`


## Individual Tasks ##
The following quick start instructions walk you through the initial process
of launching your environment and subscribing to any available task-specific messages.

### 1. Station-Keeping ###

**Summary**: Navigate to the goal pose and hold station. The best solutions will minimize the difference between the goal pose and the actual pose of the vehicle over the duration of the task.

1. Start the example: `roslaunch vrx_gazebo station_keeping.launch`
1. Subscribe to the task-specific topics provided by the stationkeeping scoring plugin:
    * The station-keeping goal (given as a [geographic_msgs/GeoPoseStamped](http://docs.ros.org/api/geographic_msgs/html/msg/GeoPoseStamped.html) message):
        * `rostopic echo /vrx/station_keeping/goal`
    * The current position error values:
        * `rostopic echo /vrx/station_keeping/pose_error`
        * `rostopic echo /vrx/station_keeping/rms_error`
1. For implementation details, see "4.1.1. Task 1: Station-Keeping" in the [Competition and Task Descriptions](https://bitbucket.org/osrf/vrx/downloads/VRX%202019%20Task%20Descriptions_v1.1.pdf), or refer to the [stationkeeping scoring plugin](https://bitbucket.org/osrf/vrx/src/default/vrx_gazebo/include/vrx_gazebo/stationkeeping_scoring_plugin.hh).

### 2. Wayfinding ###

**Summary**: Navigate through each of the published waypoints, such that vehicle achieves, as closely as possible, the positions and orientations specified.

1. Start the example: `roslaunch vrx_gazebo wayfinding.launch`
1. Subscribe to the task-specific topics provided by the wayfinding scoring plugin:
    * The list of waypoints (given as a [geographic_msgs/GeoPath](http://docs.ros.org/api/geographic_msgs/html/msg/GeoPath.html) message):
        * `rostopic echo /vrx/wayfinding/waypoints`
    * The current minimum errors achieved for each waypoint so far:
        * `rostopic echo /vrx/wayfinding/min_errors`
    * The current mean of the minimum errors: 
        * `rostopic echo /vrx/wayfinding/mean_error`
1. For implementation details, see "4.1.2. Task 2: Wayfinding" in the [Competition and Task Descriptions](https://bitbucket.org/osrf/vrx/downloads/VRX%202019%20Task%20Descriptions_v1.1.pdf), or refer to the [wayfinding scoring plugin](https://bitbucket.org/osrf/vrx/src/default/vrx_gazebo/include/vrx_gazebo/wayfinding_scoring_plugin.hh).

### 3. Perception ###

**Summary**: In this task, the vehicle remains in a fixed location and markers will appear in the field of view (see this [video](https://vimeo.com/321818142) for a demonstration of the expected behavior). The objective is to use perceptive sensors to identify the markers and report their locations.

1. Start the example: `roslaunch vrx_gazebo perception_task.launch`
1. View the camera feeds from the front of the WAM-V:
    `rosrun rqt_gui rqt_gui --perspective-file ~/vrx_ws/src/vrx/vrx_gazebo/config/front_stereo.perspective`
1. Trials will begin. Identify the type and location of the markers that appear during each trial.
1. Publish landmark identification and localization solutions as a [geographic_msgs/GeoPoseStamped](http://docs.ros.org/api/geographic_msgs/html/msg/GeoPoseStamped.html) message to the `/vrx/perception/landmark` topic:
    `rostopic pub -1 /vrx/perception/landmark geographic_msgs/GeoPoseStamped '{header: {stamp: now, frame_id: "red_mark"}, pose: {position: {latitude: 21.30996, longitude: -157.8901, altitude: 0.0}}}'`
1. Submission criteria:
    * Each trial will last for 5 seconds.
    * Solutions must be submitted before the end of the trial.
    * Only the first submission for each trial will be considered.
1. For further details, including a table of 3D objects that may appear during trials, see "4.2.3. Task 3: Landmark Localization and Characterization" in the [Competition and Task Descriptions](https://bitbucket.org/osrf/vrx/downloads/VRX%202019%20Task%20Descriptions_v1.1.pdf), or refer to the [perception scoring plugin](https://bitbucket.org/osrf/vrx/src/default/vrx_gazebo/include/vrx_gazebo/perception_scoring_plugin.hh).

### 4. Navigation Channel ###

**Summary**: Traverse a navigation channel specified by red and green markers, avoiding obstacles.

1. Start the example: `roslaunch vrx_gazebo navigation_task.launch verbose:=true`
1. There are no ROS topics specific to this task. However, relevant Gazebo messages such as "New gate crossed!" will be printed to the terminal.
1. For further details, see "4.3.1. Task 4: Traverse Navigation Channel" in the [Competition and Task Descriptions](https://bitbucket.org/osrf/vrx/downloads/VRX%202019%20Task%20Descriptions_v1.1.pdf), or refer to the [navigation scoring plugin](https://bitbucket.org/osrf/vrx/src/default/vrx_gazebo/include/vrx_gazebo/navigation_scoring_plugin.hh).

### 5 and 6: Dock and Scan-and-Dock ###

**Summary**: Given multiple docking bays, choose the correct one, dock safely, then exit the dock.

There are two variants of this challenge.

* In the first variant the correct dock is specified via a ROS message.  
* In the second variant, the correct dock must be deduced from the Scan-the-Code sequence.

Below are succinct steps to run the examples.  For more details on how successful docking is determined/scored and debugging suggestions, see [Docking Details](https://bitbucket.org/osrf/vrx/wiki/tutorials/docking_details)

#### 5. Dock: Correct Dock Specified via ROS message ####

1. Start the example: `roslaunch vrx_gazebo dock.launch verbose:=true`
1. Subscribe to the ROS topic that specifies the color and shape of the placard on the target docking bay:
    * `rostopic echo /vrx/scan_dock/placard_symbol`
1. Dock in the bay displaying the symbol published by to the `placard_symbol` topic.

If you would like to change the color and shape of the symbols there are a couple of options.  

1. If you have installed the VRX source, you can change the designated "correct" docking bay color and shape:
    1. Edit the `vrx_gazebo/worlds/dock.world.xacro` file.  
       1. Look for the `<bay>` tags and find the section with `<dock_allowed>true</dock_allowed>`
       1. Change the strings in both the `<announce_symbol>` and `<symbol>` tags to your desired color and shape, e.g., `green_triangle`.
    1. Rerun `catkin_make` to process the xacro file.
    1. Restart the example.  You should now see the target bay marked with the desired placard and you should see that the same `<COLOR>_<SHAPE>` specification is published on the  `/vrx/scan_dock/placard_symbol`.
1. Alternatively, if you would just like to change the color and shape without affecting the scoring plugin (which is how the system knows which is the "correct" bay), you can publish a ROS message to change the color and shape to a new random selection:
    * `rostopic pub /vrx/dock_2018_placard1/shuffle std_msgs/Empty "{}"` 



#### 6. Scan-and-Dock: Correct Dock Specified via Scan-the-Code ####

1. Start the example: `roslaunch vrx_gazebo scan_and_dock.launch verbose:=true`
1. View the camera feeds from the front of the WAM-V:
    `rosrun rqt_gui rqt_gui --perspective-file ~/vrx_ws/src/vrx/vrx_gazebo/config/front_stereo.perspective`
1. Approach the scan-the-code buoy and identify the sequence of three colors displayed.
1. Transmit the correct sequence to the color sequence server:
    * `rosservice call /vrx/scan_dock/color_sequence "blue" "red" "green"`
    * Notes:
        * The service name is `/vrx/scan_dock/color_sequence`
        * Allowable values are "red", "green", "blue" and "yellow"
        * The service only returns once, that is you can only call the service one time.  The first time you call the service it returns true, even if the sequence is incorrect.  If the sequence is correct, the score is incremented by the colorBonusPOints value (typ. 10 points).  You can monitor the score with `rostopic echo  /vrx/task/info` 
1. Dock in the bay displaying the symbol that corresponds to the correct color sequence.

For further details on either task, see "4.3.2. Task 5: Scan-the-code and Dock" in the [Competition and Task Descriptions](https://bitbucket.org/osrf/vrx/downloads/VRX%202019%20Task%20Descriptions_v1.1.pdf), or refer to the [scan and dock scoring plugin](https://bitbucket.org/osrf/vrx/src/default/vrx_gazebo/include/vrx_gazebo/scan_dock_scoring_plugin.hh)