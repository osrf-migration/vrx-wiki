# Phase 2 - Dress Rehearsal


For the 2019 Phase 2 - Dress Rehearsal, teams will submit their system so that it can be run against previously-unseen scenarios.
Teams have already been exposed to all of the tasks that will be present in the Dress Rehearsal scenarios.
A single system must be designed to solve any of the VRX 2019 tasks.

## What to expect

[The VRX task description](https://bitbucket.org/osrf/vrx/wiki/documentation) document has all the details about the tasks and about the environmental envelope expected in the unseen competition scenarios.

## Preparing your system

To ensure that your system can adapt to previously-unseen scenarios, teams should test their system at least against all released sample tasks.

To help you prepare, we've provided [Phase 2 Task Testing](https://bitbucket.org/osrf/vrx/wiki/Phase2_Task_Testing_2019) which includes eighteen example worlds for local testing to prepare for VRX 2019, Phase 2.  These example worlds are representative of the task configurations and environmental conditions specified in the task documentation.

For the evaluation of your Phase 2 submissions, the solutions will be run through each of these example worlds, along with some new configurations that are not released prior to the Phase 2 deadline.  The objective of this approach is to allow teams to practice with some of the actual scenarios they will see in the Phase 2 competition, so that they can compare their local performance with the results from the phase.  We also introduce new, previously-unseen scenarios, to exercise the solutions ability to deal with new configurations and conditions.

## Submission process

We expect to receive three files from each competitor prior to this event: 

* `dockerhub_image.txt`: Contains the name of the image to be pulled from DockerHub. If you're using a private DockerHub repository, please grant access to the `virtualrobotx` DockerHub user, which will be used by the organization to access your solution.
* `thruster_config.yaml`: Defines the WAM-V thruster configuration.
* `sensor_config.yaml`: Defines the WAM-V sensor configuration.

See the [submission process section](https://bitbucket.org/osrf/vrx/wiki/submission_process) in the wiki for instructions about how to submit these files.

## Important dates*****

| Date                          | Description                            |
|-------------------------------|----------------------------------------|
| 2019, October   21, 23:59 PST | Deadline for submitting your solution  |
| 2019, October   22, 23:59 PST | Deadline for submitting a correction   |
| 2019, October   28            | Results published                      |

Note: We'll allocate an extra day in the schedule (October 22) for fixing any problems related with the content of the submission files (e.g.: A noncompliant sensor or thruster configuration, a typo pointing to the DockerHub image, a problem with DockerHub permissions not letting Open Robotics to download your solution image). Teams are not allowed to modify the DockerHub image after the deadline on October 21.

*** Note:** These dates are provisional.