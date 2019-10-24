# Phase 2 Retrospective and Lessons Learned #

## Practice Worlds and Competition Worlds ##

In preparation for Phase 2 we released 3 trials for each of the 6 tasks - a total of 18 practice simulations for teams to test their solutions: [Phase 2 Task Testing](https://bitbucket.org/osrf/vrx/wiki/Phase2_Task_Testing_2019).  In running the Phase 2 Dress Rehearsal we executed the submitted code against 6 trials for each task; the first 3 trials were equivalent to the practice trials and 3 new trials that were different than the original 3 practice trials.

After the Phase 2 deadline, all of the worlds and models used for the 36 trials (6 tasks, 6 trials each) have been made a part of the VRX repository (see [PR#207](https://bitbucket.org/osrf/vrx/pull-requests/207/adding-worlds-and-models-used-in-phase2/diff)).  Following the directions in [Phase 2 Task Testing](https://bitbucket.org/osrf/vrx/wiki/Phase2_Task_Testing_2019) teams can use these trials for testing their code in many different environmental conditions.  

## Dock Task Name ##

The Task Naming in the VRX Competition and Task Descriptions document describe how the six individuals tasks (stationkeeping, wayfinding, etc.) are announced via the ROS interface.  In that document the "Dock" task corresponds to a [Task Msg](https://bitbucket.org/osrf/vrx/src/default/vrx_gazebo/msg/Task.msg) name of "scan".   This may have caused some confusion, since we probably intended to use the name "dock" to indicate this task.  However, we feel it is important to stick to the documentation as written.

For Phase 3 we will not be changing the interface, so the Dock Task (Task 5) will continue to be announced in the Task Message using the name "scan".