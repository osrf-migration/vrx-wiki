# Docking Details #

Docking is a fundamental capability that is used for both Task 5 (Dock) and Task 6 (Scan-and-Dock) of VRX.  This tutorial is meant to describe some of the details of how docking is evaluated by the VRX software to help teams develop and debug.

## Determining Success ##

The [VRX Competition and Task Descriptions](https://bitbucket.org/osrf/vrx/wiki/documentation) defines the docking sequence at a high level.  Within the VRX simulation this is implemented using the [Gazebo ContainPlugin](http://gazebosim.org/tutorials?tut=contain_plugin&cat=plugins) which generates an event when the origin of the WAM-V enters or exits a set of "activation zones".  For each docking bay, two activation zones are included as illustrated in this image:

![docking_ex.png](https://bitbucket.org/repo/BgXLzgM/images/2862828839-docking_ex.png)

The Internal Activation Zone is used to determine when the WAM-V enters or exits the dock.  The External Activation Zone is used to make sure that the WAM-V exits the bay in the proper direction.  Successful docking consists of

1.  Entering the Internal Activation Zone for one of the docking bays.  Determined by when the origin of the WAM-V is within the Internal Activation Zone geometry.  This event is reported to the stdout as a Gazebo message, e.g., `[Msg] Entering internal dock activation zone, transitioning to <docking> state in [bay1].`  This event starts a timer.
2. Staying in the Internal Activation zone for a duration of 10 seconds.  Successfully doing so is also reported to stdout, e.g., `Entering external dock activation zone in [bay1]`.  
  1. Note that if the WAM-V exits the Internal Activation Zone prior to the timer exceeding 10 seconds, the docking starts over and you get a stdout message such as `[Msg] Leaving internal dock activation zone in [bay1] after required time - transitioning to <exited> state.`
  1. Also note that if the WAM-V moves too far forward in the docking bay it may exit the Internal Activation Zone.
3. Enter External Activation Zone.  Doing so generates another stdout message, e.g., `[Msg] Entering external dock activation zone in [bay1]`
  1. This triggers a calculation of the score.  Points (10) are awarded for docking in any bay.  Additional points (10) are awarded fro docking in the correct bay as defined by the ROS topic (Task 5) or by the Scan-the-Code buoy.  Debugging messages are provide to stdout, e.g., `[Msg] Successfully docked in [bay1]. Awarding 10 points.
[Msg] Docked in incorrect dock [bay1]. No additional points.`  The score is also published to the ROS API `rostopic echo /vrx/task/info`
  1. The simulation transitions to the Finished state.

The diagram below describes this sequence as state transitions during docking maneuvers.

![docking_states.png](https://bitbucket.org/repo/BgXLzgM/images/2431361064-docking_states.png)