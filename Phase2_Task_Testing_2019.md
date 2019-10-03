# Phase 2 Task Testing 2019 #

The objective of this tutorial is to provide step-by-step guidance for local testing in preparation for [Phase 2 - Dress Rehearsal](https://bitbucket.org/osrf/vrx/wiki/events/19/dress_rehearsal).  As described in the [VRX Competition Documents](https://bitbucket.org/osrf/vrx/wiki/documentation), each of the VRX **tasks** will be evaluated over multiple **trials**, where each trial presents the task in a different configuration (e.g., a different waypoint location) and with different environmental conditions (wind, waves, lighting, etc.)  The specifications of the environmental envelope (the range of possible environmental parameters) is included in VRX Technical Guide and the specifications of each task are in the VRX Competition and Task Descriptions - both documents are available on the [VRX Competition Documents wiki](https://bitbucket.org/osrf/vrx/wiki/documentation)

The [VRX Tasks: Examples](https://bitbucket.org/osrf/vrx/wiki/tutorials/vrx_tasks_2019) wiki provides general instructions and example of a single trial for each task, which is a great place to start.  However, if your solution is going to do well in the competition, it should be able to perform over a wide range of task and environment conditions.  The purpose of this tutorial is to show you... 

 1. How to locally test each task with multiple trials
 1. How to evaluate and verify your performance

For this tutorial, all of your testing will be done locally, on the host machine or a local docker container.  For the actual competition, these solutions will be evaluated automatically.  If you are interested in testing that aspect of the evaluation, you can setup your own evaluation setup, equivalent to the one used in the competition, using the tools describe in the [vrx-docker repository](https://bitbucket.org/osrf/vrx-docker/src/default/).


## Example Trials ##

We have generated three trials for each task that cover much of the allowable task and environment parameters.   These trials are notionally

0. Easy - simplified task in negligible wave, wind and visual (fog) environmental factors.
1. Medium - moderate task difficulty and environmental influence
2. Hard - at or close to the limit of task difficulty and environmental factors.

Our intention is to execute the evaluation of submissions to the Phase 2 challenge in using very similar (but not exactly the same) trials of each task.  Each trial consists of worlds and models to define the instance of the task and the operating environment.

### Running Example Trials ###
All examples are stored in the `vrx/vrx_gazebo/worlds` directory.

You should be able to run the individual examples as

```
WORLD=stationkeeping0.world
roslaunch vrx_gazebo vrx.launch verbose:=true \
	  paused:=false \
	  wamv_locked:=true \
	  world:=/home/developer/vrx_ws/src/vrx/vrx_gazebo/worlds/${WORLD}
```
where you will want to change the value of the `WORLD` variable to match each of the worlds you want to run.


## Task 1: Stationkeeping ##
The three example worlds provided for the Stationkeeping task are as follows:

* `stationkeeping0.world`: Easy environment.
* `stationkeeping1.world`: Medium difficulty environment.
* `stationkeeping2.world`: Hard difficulty environment.  

### Stationkeeping World Variations ###

These three worlds all have goals set relatively close to the WAM-V starting position. Due to the nature of the task, they vary primarily in terms of environmental factors.

## Task 2: Wayfinding ##
The three example worlds provided for the Wayfinding task are as follows:

* `wayfinding0.world`: Easy environment. Three waypoints, relatively close together.
* `wayfinding1.world`: Medium difficulty environment. Four waypoints, more widely dispersed.
* `wayfinding2.world`: Hard difficulty environment.  Five waypoints in a more challenging arrangement.

See below for screenshots showing the arrangement of waypoints in each example world.

### wayfinding0.world ###

Easy environment. Three waypoints, relatively close together.

![wayfinding0.png](https://bitbucket.org/repo/BgXLzgM/images/1116678533-wayfinding0.png)

### wayfinding1.world ###

Medium difficulty environment. Four waypoints, more widely dispersed.

![wayfinding1.png](https://bitbucket.org/repo/BgXLzgM/images/33173320-wayfinding1.png)

### wayfinding2.world ###

Hard difficulty environment.  Five waypoints in a more challenging arrangement.

![wayfinding2.png](https://bitbucket.org/repo/BgXLzgM/images/1278469224-wayfinding2.png)

## Task 5: Dock ##

Three example worlds are supplied to represent three potential trials of the Dock task.

* `dock0.world`: Easy environment.  Correct bay is the blue_circle directly ahead of the WAM-V in its initial position.
* `dock1.world`: Medium difficulty environment.  Correct bay is the green_triangle.
* `dock2.world`: Hard difficulty environment.  There are two docks (four bays).  The correct bay is red_triangle.

### dock0.world ###

Easy environment.  Correct bay is the blue_circle directly ahead of the WAM-V in its initial position.

![dock0.png](https://bitbucket.org/repo/BgXLzgM/images/81999219-dock0.png)

### dock1.world ###

 Medium difficulty environment.  Correct bay is the green_triangle.

![dock1.png](https://bitbucket.org/repo/BgXLzgM/images/3348985337-dock1.png)

### dock2.world ###

Hard difficulty environment.  There are two docks (four bays).  The correct bay is red_triangle.

![dock2.png](https://bitbucket.org/repo/BgXLzgM/images/448798662-dock2.png)

### Development and Debugging Recommendations ###

The [VRX Tasks: Examples](https://bitbucket.org/osrf/vrx/wiki/tutorials/vrx_tasks_2019#markdown-header-5-and-6-dock-and-scan-and-dock) wiki provides general notes on the API and [Docking Details](https://bitbucket.org/osrf/vrx/wiki/tutorials/docking_details) wiki describes the how successful docking is determined and how the docking state is provided to stdout.

## Task 6: Scan-and-Dock ##

The three example trials for this task have the same environmental and dock layout of Task 5: Dock, described above.  In each example world a RoboX Light Buoy is added and the system must find the light buoy, read the code, and infer the correct dock from the light sequence.  

* `scan_and_dock0.world`: Easy environment.  Correct bay is the blue_circle directly ahead of the WAM-V in its initial position.  Light buoy is immediately visible.
* `scan_and_dock1.world`: Medium difficulty environment.  Correct bay is the green_triangle.
* `scan_and_dock2.world`: Hard difficulty environment.  There are two docks (four bays).  The correct bay is red_triangle.

### scan_and_dock0.world ###

![scandock0.png](https://bitbucket.org/repo/BgXLzgM/images/2522946443-scandock0.png)

### scan_and_dock1.world ###

![scandock1.png](https://bitbucket.org/repo/BgXLzgM/images/2101582440-scandock1.png)

### scan_and_dock2.world ###

![scandock2.png](https://bitbucket.org/repo/BgXLzgM/images/426771738-scandock2.png)