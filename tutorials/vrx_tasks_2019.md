# VRX Tasks: Examples #

[TOC]

The individual tasks for the 2019 VRX competition (link to document coming soon!) are supported by example simulation worlds and Gazebo plugins to evaluate and score task performance.  Below are examples that illustrate the implementation of the these tasks.  For descriptions of the task, the application interface (API) for each task and the scoring, please see the Description of Tasks and Technical Guide documents (again, links coming soon).

## General Instructions ##

### Monitoring Task Status ###

The [vrx_gazebo/Task](https://bitbucket.org/osrf/vrx/src/default/vrx_gazebo/msg/Task.msg) status message includes, task state {Initial, Ready, Running, Finished}, current score and timing information (details of the API are in the Technical Guide (link coming soon)).  Recommended that you monitor the status during simulation, e.g.,

```
rostopic echo /vrx/task/info 
```

## Station-Keeping ##

1. Start the Gazebo example: `roslaunch vrx_gazebo station_keeping.launch`



## Wayfinding ##

## Perception ##

## Navigation Channel ##

## Scan-the-Code and Dock ##