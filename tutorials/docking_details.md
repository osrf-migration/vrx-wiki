# Docking Details #

Docking is a fundamental capability that is used for both Task 5 (Dock) and Task 6 (Scan-and-Dock) of VRX.  This tutorial is meant to describe some of the details of how docking is evaluated by the VRX software to help teams develop and debug.

## Determining Success ##

The [VRX Competition and Task Descriptions](https://bitbucket.org/osrf/vrx/wiki/documentation) defines the docking sequence at a high level.  Within the VRX simulation this is implemented using the [Gazebo ContainPlugin](http://gazebosim.org/tutorials?tut=contain_plugin&cat=plugins) which generates an event when the origin of the WAM-V enters or exits a set of "activation zones".  For each docking bay, two activation zones are included as illustrated in this image:

![docking_ex.png](https://bitbucket.org/repo/BgXLzgM/images/2862828839-docking_ex.png)

The Internal Activation Zone is used to determine when the WAM-V enters or exits the dock.  The External Activation Zone is used to make sure that the WAM-V exits the bay in the proper direction.
The diagram below describes the state transitions during docking maneuvers.
![docking_states.png](https://bitbucket.org/repo/BgXLzgM/images/2431361064-docking_states.png)