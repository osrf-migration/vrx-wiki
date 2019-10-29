# Summary
This tutorial will allow you to verify that your submission is working as expect.

# Prerequisites
We assume the following:

* You have a local installation of the vrx environment built from source according to the [host-based installation instructions (option 2)](https://bitbucket.org/osrf/vrx/wiki/tutorials/SystemSetupInstall).
* Docker is [installed on your system](https://docs.docker.com/install/linux/docker-ce/ubuntu/).
* You have developed a solution to one or more VRX tasks.
* You have [created a Docker image containing your solution](https://bitbucket.org/osrf/vrx/wiki/tutorials/Creating%20a%20Dockerhub%20image%20for%20submission) and uploaded it to Docker hub.
* You have created the [required files](https://bitbucket.org/osrf/vrx/wiki/events/19/vrx_challenge) for a submission.
* You have [forked and cloned the vrx-events repository](https://bitbucket.org/osrf/vrx/wiki/submission_process) to your home directory and added your files.

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

3 . Source the your `bash.setup` file, change into the `vrx-docker` directory, and set the variable `TEAM` for later use:
```bash
source ~/vrx_ws/devel/setup.bash
cd vrx-docker
TEAM=<your_team_name>
```
Replace <your_team_name> with the team name you used when adding your submission files to the `vrx-events` directory.

4 . Copy your submission files from your local fork of the vrx-events repository to the `vrx-docker/team_config` folder.

```bash
cp -R ~/vrx-events/2019/phase3_vrx_challenge/$TEAM team_config/
```

4 . Build the vrx-server docker image (may take 30-60 minutes the first time):
```bash
./vrx_server/build_image.bash -n
```

**Note:** The above command expects a system with an Nvidia graphics card. To build an image on a system without Nvidia graphics, remove the `-n` option. Without an Nvidia card, the simulation is likely to run slower than real-time. 

# Verify your `dockerhub_image.txt` file.

1 . Test that your `dockerhub_image.txt` file contains the name of a reachable docker image (and version):

```
#!bash
    
cat team_config/$TEAM/dockerhub_image.txt | xargs docker pull
```

2 . If the contents of the file are correct, docker should begin to pull your image. Once you have verified this is working, you can exit out of the pull using `ctrl+c`.

# Verify your sensor configuration.



# Verify your docker image is working
After completing the steps in the sections above