# Overview

This tutorial outlines the process for creating a Dockerhub image to be submitted for the VRX competition. This is required for creating a container that will run a VRX team's system to control their virtual autonomous boat for the automated evaluation.

# What is Docker 

A Docker image is an executable package that includes everything needed to run an application--the code, a runtime, libraries, environment variables, and configuration files. A Docker container is a runtime instance of an image. Without Docker, evaluating a VRX team's software on different machines would require that the environment matches perfectly for the VRX team's system to run as expected, which is very prone to error. With Docker, if a VRX team's system works reliably on their machine, it will work reliably on any machine. 

To learn more about what Docker images and containers are, Docker has excellent [tutorials and explanations](https://docs.docker.com/get-started/).

# Quick Start Instructions:

* Docker is required to create a Dockerhub image. Please follow the [VRX Docker install tutorial](https://bitbucket.org/osrf/vrx/wiki/tutorials/installDocker) and the [Nvidia Docker install tutorial](https://bitbucket.org/osrf/vrx/wiki/tutorials/installNvidiaDocker) if you are
using an Nvidia GPU.

* Create a Dockerhub account [here](https://hub.docker.com/signup) if you do not already have one. Take note of your username.

* From here, there are two main options to create your Docker image. The first option is simpler and more intuitive for first-time Docker users. The second option is cleaner and less prone to error, but is more advanced.

## Option 1: Pull the ROS Melodic Docker image and manually run desired commands to setup the system

* Run `docker run --name my_container -it ros:melodic-ros-base`. This will pull the `ros:melodic-ros-base` image from DockerHub and create a container of that image called `my_container. `-it` is added so that when this is complete, it starts an interactive Bash session for you to run commands. This may take a few minutes to run.

* This Bash session is very barebones. It does not have a text editor yet, so we will install one now. From the created interactive Bash session, run `apt-get update && apt-get install -y nano` or replace `nano` with your text editor of choice.  

* Use the text editor to edit `ros_entrypoint.sh`. Eg. `nano /ros_entrypoint.sh`. This is a script that is run immediately after your container has been built. Replace all the text with the following:

```
#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

/run_my_system.bash
```

This will run the script `run_my_system.bash`. 

* Create the `run_my_system.bash` script. `nano /run_my_system.bash`. Copy the following text into the file

```
#!/bin/bash

# Create ros master if not already started
rostopic list > /dev/null 2>&1
retVal=$?
if [ $retVal -ne 0 ]; then
    roscore &
    echo "Wait for 5s to allow rosmaster to start"
    sleep 5s
else
    echo "rosmaster already setup"
fi

# Send forward command
RATE=1
CMD=2
echo "Sending forward command"
rostopic pub /wamv/thrusters/left_thrust_cmd std_msgs/Float32 -r ${RATE} -- ${CMD} &
rostopic pub /wamv/thrusters/right_thrust_cmd std_msgs/Float32 -r ${RATE} -- ${CMD}
```

* Run `chmod +x /run_my_system.bash` to make it executable.

* Run `/run_my_system.bash && rostopic echo /wamv/thrusters/left_thrust_cmd` to test if the script works. You should receive `/wamv/thrusters/left_thrust_cmd` data.

* Run `exit` to leave this container.

* Run `docker ps -a` to list all containers. You should see your container `my_container`.

* Copy the Container ID of `my_container`.

* Run `docker commit -m "Start off ros-melodic-base, add run_my_system simple script" -a "<FULL NAME>" <Container_ID> <USERNAME>/<IMAGE_REPOSITORY_NAME>:<TAG>`. Your username must match the username of your Dockerhub account. The image repository name is the repository name that the image will be saved to on Dockerhub. The tag and colon are optional (but highly recommended) to help you version your images. Example: `docker commit -m "Start off ros-melodic-base, add run_my_system simple script" -a "Tyler Lum" a0e1e92cb6a5 tylerlum/vrx-competitor-example:v2.2019` or `docker commit -m "Start off ros-melodic-base, add run_my_system simple script" -a "Tyler Lum" a0e1e92cb6a5 tylerlum/vrx-competitor-example`

* Run `docker login`

* Run `docker push <USERNAME>/<IMAGE_REPOSITORY_NAME>:<TAG>` with the same information as the previous step. Eg. `docker push tylerlum/vrx-competitor-example:v2.2019`

* You should be able to log onto your Dockerhub account and see your new repository. 

* Optional: If you want to keep your repository private, you can click on your repository, then click Settings, then Make Private. To ensure that your Docker image can be evaluated, you can click Collaborators and add the desired Docker ID. Exact details about submission and Docker ID are coming soon. 

## Option 2: Create a Dockerfile and entrypoint script 

* Run `mkdir ~/my_vrx_docker; cd ~/my_vrx_docker`

* Run `gedit Dockerfile` and copy the following text into it:

```
# This is an auto generated Dockerfile for ros:ros-base
# generated from docker_images/create_ros_image.Dockerfile.em
FROM ros:melodic-ros-core-bionic

# install ros packages
RUN apt-get update && apt-get install -y \
    ros-melodic-ros-base=1.4.1-0* \
&& rm -rf /var/lib/apt/lists/*

# Copy over script to Docker container
COPY ./run_my_system.bash /

# Use your ros_entrypoint
COPY ./ros_entrypoint.sh /
```

* Run `gedit run_my_system.sh` to create your script. Copy the following text into the file

```
#!/bin/bash

# Create ros master if not already started
rostopic list > /dev/null 2>&1
retVal=$?
if [ $retVal -ne 0 ]; then
    roscore &
    echo "Wait for 5s to allow rosmaster to start"
    sleep 5s
else
    echo "rosmaster already setup"
fi

# Send forward command
RATE=1
CMD=2
echo "Sending forward command"
rostopic pub /left_thrust_cmd std_msgs/Float32 -r ${RATE} -- ${CMD} &
rostopic pub /right_thrust_cmd std_msgs/Float32 -r ${RATE} -- ${CMD}
```

Then run `chmod +x run_my_system.bash` to make it executable.

* Run `gedit ros_entrypoint.sh` to create your script. Copy the following text into the file

```
#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

/run_my_system.bash
```

Then run `chmod +x ros_entrypoint.sh` to make it executable.
* Run `docker build --tag <USERNAME>/<IMAGE_REPOSITORY_NAME>:<TAG> .` Eg. `docker build --tag tylerlum/vrx-competitor-example:v2.2019 .`

* Run `docker run <USERNAME>/<IMAGE_REPOSITORY_NAME>:<TAG>` This will create a container with the image you created in the previous step, and then run `/run_my_system.bash`.

* Run `docker ps -a` and take note of your container id. Then run `docker commit -m "Start off ros-melodic-base, add run_my_system simple script" -a "<FULL NAME>" <Container_ID> <USERNAME>/<IMAGE_REPOSITORY_NAME>:<TAG>`. Your username must match the username of your Dockerhub account. The image repository name is the repository name that the image will be saved to on Dockerhub. The tag and colon are optional to help you version your images. Example: `docker commit -m "Start off ros-melodic-base, add run_my_system simple script" -a "Tyler Lum" a0e1e92cb6a5 tylerlum/vrx-competitor-example:v2.2019` or `docker commit -m "Start off ros-melodic-base, add run_my_system simple script" -a "Tyler Lum" a0e1e92cb6a5 tylerlum/vrx-competitor-example`

* Run `docker login`

* Run `docker push <USERNAME>/<IMAGE_REPOSITORY_NAME>:<TAG>` with the same information as the previous step. Eg. `docker push tylerlum/vrx-competitor-example:v2.2019`

* You should be able to log onto your Dockerhub account and see your new repository. 

* Optional: If you want to keep your repository private, you can click on your repository, then click Settings, then Make Private. To ensure that your Docker image can be evaluated, you can click Collaborators and add the desired Docker ID. Exact details about submission and Docker ID are coming soon. 

# Working with your container

To see what is inside of your container, you can run:

```
docker run --name my_container <USERNAME>/<IMAGE_REPOSITORY_NAME>:<TAG>
```

Then in a different terminal, you can run:

```
docker exec -it my_container bash
```

This allows you to enter the running container and investigate:

```
source /opt/ros/melodic/setup.bash
rostopic list

/left_thrust_cmd
/right_thrust_cmd
/rosout
/rosout_agg

rostopic echo /left_thrust_cmd

data: 2.0
---
data: 2.0
---

```

You can exit with 

```
exit
``` 

To kill the container from outside the container, you can run:

```
docker kill my_container
```

# Description

More details coming soon.