# Phase 2 Retrospective and Lessons Learned #

## Practice Worlds and Competition Worlds ##

In preparation for Phase 2 we released 3 trials for each of the 6 tasks - a total of 18 practice simulations for teams to test their solutions: [Phase 2 Task Testing](https://bitbucket.org/osrf/vrx/wiki/Phase2_Task_Testing_2019).  In running the Phase 2 Dress Rehearsal we executed the submitted code against 6 trials for each task; the first 3 trials were equivalent to the practice trials and 3 new trials that were different than the original 3 practice trials.

After the Phase 2 deadline, all of the worlds and models used for the 36 trials (6 tasks, 6 trials each) have been made a part of the VRX repository (see [PR#207](https://bitbucket.org/osrf/vrx/pull-requests/207/adding-worlds-and-models-used-in-phase2/diff)).  Following the directions in [Phase 2 Task Testing](https://bitbucket.org/osrf/vrx/wiki/Phase2_Task_Testing_2019) teams can use these trials for testing their code in many different environmental conditions.  See descriptions of the [Phase 2 Trials](https://bitbucket.org/osrf/vrx/wiki/events/19/phase2_trials).

## P3D Gazebo Plugin ##

Going through the submissions for Phase 2, we noticed that the teams include the P3D plugin in their sensor_config.yaml file. The P3D plugin provides a ROS published ground truth - true, noiseless pose information from Gazebo.  During development it is often handy to know the true state of the system, but the system should estimate the state of the system from sensors - not use the ground truth directly in the solution.  

This "sensor" is not included in the Technical Guide, but the [Sensor Configuration Wiki ](https://bitbucket.org/osrf/vrx/wiki/tutorials/Creating%20a%20custom%20WAM-V%20Thruster%20and%20Sensor%20Configuration%20For%20Competition) points to the `numeric.yaml` file which could be interpreted to suggest that a sensor configuration that includes one P3D sensor is compliant.

For Phase 3 including the P3D sensor in the `sensor_config.yaml` file will result in a non-compliant configuration.

## Dock Task Name ##

The Task Naming in the VRX Competition and Task Descriptions document describe how the six individuals tasks (stationkeeping, wayfinding, etc.) are announced via the ROS interface.  In that document the "Dock" task corresponds to a [Task Msg](https://bitbucket.org/osrf/vrx/src/default/vrx_gazebo/msg/Task.msg) name of "scan".   This may have caused some confusion, since we probably intended to use the name "dock" to indicate this task.  However, we feel it is important to stick to the documentation as written.

For Phase 3 we will not be changing the interface, so the Dock Task (Task 5) will continue to be announced in the Task Message using the name "scan".