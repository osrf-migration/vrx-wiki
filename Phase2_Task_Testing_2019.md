# Phase 2 Task Testing 2019 #

The objective of this tutorial is to provide step-by-step guidance for local testing in preparation for [Phase 2 - Dress Rehearsal](https://bitbucket.org/osrf/vrx/wiki/events/19/dress_rehearsal).  As described in the [VRX Competition Documents](https://bitbucket.org/osrf/vrx/wiki/documentation), each of the VRX **tasks** will be evaluated over multiple **trials**, where each trial presents the task in a different configuration (e.g., a different waypoint location) and with different environmental conditions (wind, waves, lighting, etc.)  The specifications of the environmental envelope (the range of possible environmental parameters) is included in VRX Technical Guide and the specifications of each task are in the VRX Competition and Task Descriptions - both documents are available on the [VRX Competition Documents wiki](https://bitbucket.org/osrf/vrx/wiki/documentation)

The [VRX Tasks: Examples](https://bitbucket.org/osrf/vrx/wiki/tutorials/vrx_tasks_2019) wiki provides general instructions and example of a single trial for each task, which is a great place to start.  However, if your solution is going to do well in the competition, it should be able to perform over a wide range of task and environment conditions.  The purpose of this tutorial is to show you... 

 1. How to locally test each task with multiple trials
 1. How to evaluate and verify your performance

For this tutorial, all of your testing will be done locally, on the host machine or a local docker container.  For the actual competition, these solutions will be evaluated automatically.  If you are interested in testing that aspect of the evaluation, you can setup your own evaluation setup, equivalent to the one used in the competition, using the tools describe in the [vrx-docker repository](https://bitbucket.org/osrf/vrx-docker/src/default/).


## Example Worlds ##

We have generated three trials for each task that cover much of the allowable task and environment parameters.   These trials are notionally

0. Easy - simplified task in negligible wave, wind and visual environmental factors.
1. Medium - moderate task difficulty and environmental influence
2. Hard - at or close to the limit of task difficulty and environmental factors.

Our intention is to execute the evaluation of submissions to the Phase 2 challenge in using very similar (but not exactly the same) trials of each task.