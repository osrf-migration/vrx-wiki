# Phase 3 - VRX Challenge #
The process and content of the Phase 3 will be very similar to the [Phase 2: Dress Rehearsal](https://bitbucket.org/osrf/vrx/wiki/events/19/dress_rehearsal).

## Lessons Learned ##

Please review our [Phase 2 Retrospective and Lessons Learned](https://bitbucket.org/osrf/vrx/wiki/events/19/phase2_retrospective) and the [Phase 2 Results](https://bitbucket.org/osrf/vrx/wiki/VRX%202019%20Results) for notes and feedback from the Phase 2 Dress Rehearsal.

## What to expect ##

For the purposes of continued development, teams can expect that the submission process will be the same as Phase 2.  We will only introduce changes to incorporate lessons learned from Phase 2.  

The VRX Tasks will be be the same as provided in the [VRX task description document](https://bitbucket.org/osrf/vrx/wiki/documentation).  Just as done in Phase 2, team performance for these tasks will be assessed by running the team software on multiple trials of each task, where each trial instantiates a specific task scenario consistent with the general task description.

## Preparing your system ##

To ensure that your system can adapt to previously-unseen scenarios, teams should test their system at least against all released sample tasks.

To help you prepare, we've provided [Phase 2 Task Testing](https://bitbucket.org/osrf/vrx/wiki/Phase2_Task_Testing_2019) which includes thirty-six example worlds for local testing to prepare for VRX 2019, Phase 2 and Phase 3. These example worlds are representative of the task configurations and environmental conditions specified in the task documentation.

For the evaluation of your Phase 3 submissions, the solutions will be run through new configurations that are not released prior to the Phase 3 deadline, to exercise the solutions ability to deal with new configurations and conditions.

## Submission process

We expect to receive three files from each competitor prior to this event: 

* `dockerhub_image.txt`: Contains the name of the image to be pulled from DockerHub. 
* `thruster_config.yaml`: Defines the WAM-V thruster configuration.
* `sensor_config.yaml`: Defines the WAM-V sensor configuration.

### Submission process notes

* **If you're using a private DockerHub repository, please grant access to the `virtualrobotx` DockerHub user, which will be used by the organization to access your solution.** 
* Follow [this tutorial](https://bitbucket.org/osrf/vrx/wiki/tutorials/Creating%20a%20Dockerhub%20image%20for%20submission) for instructions about how to create your own Docker image.
* Check out [this tutorial](https://bitbucket.org/osrf/vrx/wiki/Testing%20your%20submission) to learn how to verify your submission.
* See the [submission process section](https://bitbucket.org/osrf/vrx/wiki/submission_process) in the wiki for instructions about how to submit these files.

Once you submit the pull request, the VRX technical team will do two things before merging (accepting) the submission:

1. Check that the WAM-V thruster and sensor configuration complies with the configuration constraints described in the VRX Technical Guide
2. Check that the DockerHub image is accessible by the virtualrobotx DockerHub user.

Once those two requirements are met, the pull request will be merged and your submission is considered ready.

## Important dates

| Date                          | Description                            |
|-------------------------------|----------------------------------------|
| 2019, November 22, 23:59 PST  | Deadline for submitting your solution  |
| 2019, November 25, 23:59 PST  | Deadline for submitting a correction   |
| 2019, November 29             | Results published                      |

Note: We'll allocate an extra day in the schedule (November 25) for fixing any problems related with the content of the submission files (e.g.: A noncompliant sensor or thruster configuration, a typo pointing to the DockerHub image, a problem with DockerHub permissions not letting Open Robotics to download your solution image). Teams are not allowed to modify the DockerHub image after the deadline on November 22.