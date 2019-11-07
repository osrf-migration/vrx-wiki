# Summary
This tutorial will allow you to verify that your submission is working as expected.

# Prerequisites
We assume the following:

* You have a local installation of the vrx environment built from source according to the [host-based installation instructions (option 2)](https://bitbucket.org/osrf/vrx/wiki/tutorials/SystemSetupInstall).
* Docker is [installed on your system](https://docs.docker.com/install/linux/docker-ce/ubuntu/).
* You have developed a solution to one or more VRX tasks.
* You have [created a Docker image containing your solution](https://bitbucket.org/osrf/vrx/wiki/tutorials/Creating%20a%20Dockerhub%20image%20for%20submission) and uploaded it to Docker hub.
* You have created the [required files](https://bitbucket.org/osrf/vrx/wiki/events/19/vrx_challenge) for a submission.
* You have [forked and cloned the vrx-events repository](https://bitbucket.org/osrf/vrx/wiki/submission_process) to your home directory and added your files.
* You are running on a system with an Nvidia graphics card. If this is not the case, you can still test your submission by leaving out the `-n` option wherever it appears below. Note, however, that without an Nvidia card, the simulation is likely to run slower than real-time. 

# Prepare a local testing environment

1 . Change into the `vrx_ws/src` directory you created when installing `vrx`.

```bash
cd ~/vrx_ws/src
```

2 . Clone the `vrx-docker` repository.

```bash
hg clone https://bitbucket.org/osrf/vrx-docker
```

This should create a new `vrx-docker` directory alongside the original `vrx` repository directory.

3 . Source your `bash.setup` file, change into the `vrx-docker` directory, and set the variable `TEAM` for later use:
```bash
source ~/vrx_ws/devel/setup.bash
cd vrx-docker
TEAM=<your_team_name>
```
Replace <your_team_name> with the team name you used when adding your submission files to the `vrx-events` directory.

4 . Copy your submission files (yaml config files and docker hub image name) from your local fork of the vrx-events repository to the `vrx-docker/team_config` folder.  For example, if you are testing the submission files for the [Phase 3 VRX Challenge](https://bitbucket.org/osrf/vrx/wiki/events/19/vrx_challenge) in the [vrx-events](https://bitbucket.org/osrf/vrx-events/src/default/) repository

```bash
cp -R "~/vrx-events/2019/phase3_vrx_challenge/$TEAM" team_config/
```

4 . Build the vrx-server docker image (may take 30-60 minutes the first time):
```bash
./vrx_server/build_image.bash -n
```

# Verify your dockerhub_image.txt file.

1 . Test that your dockerhub_image.txt file contains the name of a reachable docker image (and version):

```
#!bash
    
cat "team_config/$TEAM/dockerhub_image.txt" | xargs docker pull
```

2 . If the contents of the file are correct, docker should begin to pull your image. Once you have verified this is working, you can exit out of the pull using `ctrl+c`.

# Check propulsion and sensor compliance.
1 . Run the prepare_team_wamv.bash script included with `vrx-docker` to set up your team's wamv configuration: 
```bash
./prepare_team_wamv.bash "$TEAM"
```
Note that this will produce a `REQUIRED process [wamv_config/wamv_generator-2] has died!` message, which is expected.

2 . Check compliance:
```bash
cat "generated/team_generated/$TEAM/compliant.txt"
```
The output of the above should be **true** if your configuration passes compliance tests.

# Verify your docker image is working
1 . Define a TASK variable to indicate the task you wish to test against. 
```bash
TASK=stationkeeping
```
Other valid options include `wayfinding`, `perception`, `nav_challenge`, `dock`, and `scan_and_dock`.

2 . Build example task worlds using the `prepare_task_trials.bash` script included with `vrx-docker`:
```bash
./prepare_task_trials.bash $TASK
```

3 . Define a TRIAL variable to indicate which of the generated task worlds you would like to run.
```bash
TRIAL=0
```

4 . Use the `run_trial.bash` script to test your submission on the task and trial specified above:
```bash
./run_trial.bash -n $TEAM $TASK $TRIAL
```
This command will run your submission image and the `vrx-server` image at the same time and generate multiple log files which are saved in the `vrx-docker/generated/logs` directory.

5 . View your score for the task:
```bash
cat generated/logs/$TEAM/$TASK/$TRIAL/trial_score.txt
```

6 . See a replay of your system's performance on the task and verify that it behaved as you expected:
```bash
roslaunch vrx_gazebo playback.launch log_file:=`pwd`/generated/logs/$TEAM/$TASK/$TRIAL/gazebo-server/state.log
```