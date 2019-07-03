# Overview

This tutorial outlines the process for creating a Dockerhub image to be submitted for the VRX competition. This is required for creating a container that will run a VRX team's system to control their virtual autonomous boat for the automated evaluation.

A Docker image is an executable package that includes everything needed to run an application--the code, a runtime, libraries, environment variables, and configuration files. A Docker container is a runtime instance of an image. Without Docker, evaluating a VRX team's software on different machines would require that the environment matches perfectly for the VRX team's system to run as expected, which is very prone to error. With Docker, if a VRX team's system works reliably on their machine, it will work reliably on any machine. 

To learn more about what Docker images and containers are, Docker has excellent [tutorials and explanations](https://docs.docker.com/get-started/).

# Quick Start Instructions:

1. Docker is required to create a Docker image and run the automated evaluation. Please follow the [Docker CE for Ubuntu tutorial's](https://docs.docker.com/install/linux/docker-ce/ubuntu) __Prerequisites__ and __Install Docker CE__ sections.

2. Continue to the [Docker post-install instructions](https://docs.docker.com/engine/installation/linux/linux-postinstall/) and complete the __Manage Docker as a non-root user__ section to avoid having to run the commands on this page using `sudo`.

3. Create a Dockerhub account [here](https://hub.docker.com/signup) if you do not already have one. Take note of your username.

From here, there are two main options to create your Docker image. The first option is simpler and more intuitive for first-time Docker users. The second option is cleaner and less prone to error, but is more advanced.

## Option 1: Pull the ROS Melodic Docker image and manually run desired commands to setup the system

1. Run `docker run --name my_container -it ros:melodic-ros-base`. This will pull the `ros:melodic-ros-base` image from DockerHub and create a container of that image called `my_container. `-it` is added so that when this is complete, it starts an interactive Bash session for you to run commands. This may take a few minutes to run.

2. This Bash session is very barebones. It does not have a text editor yet, so we will install one now. From the created interactive Bash session, run `apt-get update && apt-get install -y nano` or replace `nano` with your text editor of choice.  

3. Use the text editor to edit `ros_entrypoint.sh`. Eg. `nano /ros_entrypoint.sh`. This is a script that is run immediately after your container has been built. Replace all the text with the following:

```
#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

/run_my_system.bash
```

This will run the script `run_my_system.bash`. 

4. Create the `run_my_system.bash` script. `nano /run_my_system.bash`. Copy the following text into the file

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
rostopic pub /left_thrust_cmd std_msgs/Float32 -r ${RATE} -- ${CMD} &
rostopic pub /right_thrust_cmd std_msgs/Float32 -r ${RATE} -- ${CMD} &
echo "Sending forward command"
```

5. Run `chmod +x /run_my_system.bash` to make it executable.

6. Run `/run_my_system.bash && rostopic echo /left_thrust_cmd` to test if the script works. You should receive `/left_thrust_cmd` data.

7. Run `exit` to leave this container.

8. Run `docker ps -a` to list all containers. You should see your container `my_container`.

9. Copy the Container ID of `my_container`.

10. Run `docker commit -m "Start off ros-melodic-base, add run_my_system simple script" -a "<FULL NAME>" <Container_ID> <USERNAME>/<IMAGE_REPOSITORY_NAME>:<TAG>`. Your username must match the username of your Dockerhub account. The image repository name is the repository name that the image will be saved to on Dockerhub. The tag and colon are optional to help you version your images. Example: `docker commit -m "Start off ros-melodic-base, add run_my_system simple script" -a "Tyler Lum" a0e1e92cb6a5 tylerlum/vrx-competitor-example:v1.2019` or `docker commit -m "Start off ros-melodic-base, add run_my_system simple script" -a "Tyler Lum" a0e1e92cb6a5 tylerlum/vrx-competitor-example`

11. Run `docker login`

12. Run `docker push <USERNAME>/<IMAGE_REPOSITORY_NAME>:<TAG>` with the same information as the previous step. Eg. `docker push tylerlum/vrx-competitor-example:v1.2019`

13. You should be able to log onto your Dockerhub account and see your new repository. 

14. Optional: If you want to keep your repository private, you can click on your repository, then click Settings, then Make Private. To ensure that your Docker image can be evaluated, you can click Collaborators and add the desired Docker ID. Exact details about submission and Docker ID are coming soon. 

## Option 2: Create a Dockerfile and entrypoint script 
1. Run `mkdir ~/my_vrx_docker; cd ~/my_vrx_docker`

2. Run `gedit Dockerfile` and copy the following text into it:

```
# This is an auto generated Dockerfile for ros:ros-base
# generated from docker_images/create_ros_image.Dockerfile.em
FROM ros:melodic-ros-core-bionic

# install ros packages
RUN apt-get update && apt-get install -y \
    ros-melodic-ros-base=1.4.1-0* \
&& rm -rf /var/lib/apt/lists/*

COPY ./run_my_system.bash /
```

3. Run `gedit run_my_system.bash` to create your script. Copy the following text into the file

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
rostopic pub /left_thrust_cmd std_msgs/Float32 -r ${RATE} -- ${CMD} &
rostopic pub /right_thrust_cmd std_msgs/Float32 -r ${RATE} -- ${CMD} &
echo "Sending forward command"
```

Then run `chmod +x run_my_system.bash` to make it executable.

4. Run `docker build --tag <USERNAME>/<IMAGE_REPOSITORY_NAME>:<TAG> .` Eg. `docker build --tag tylerlum/vrx-competitor-example:v1.2019 .`

5. Run `docker run <USERNAME>/<IMAGE_REPOSITORY_NAME>:<TAG> "/run_my_system.bash"` This will create a container with the image you created in the previous step, and then run `/run_my_system.bash`.

6. Run `docker ps -a` and take note of your container id. Then run `docker commit -m "Start off ros-melodic-base, add run_my_system simple script" -a "<FULL NAME>" <Container_ID> <USERNAME>/<IMAGE_REPOSITORY_NAME>:<TAG>`. Your username must match the username of your Dockerhub account. The image repository name is the repository name that the image will be saved to on Dockerhub. The tag and colon are optional to help you version your images. Example: `docker commit -m "Start off ros-melodic-base, add run_my_system simple script" -a "Tyler Lum" a0e1e92cb6a5 tylerlum/vrx-competitor-example:v1.2019` or `docker commit -m "Start off ros-melodic-base, add run_my_system simple script" -a "Tyler Lum" a0e1e92cb6a5 tylerlum/vrx-competitor-example`

7. Run `docker login`

8. Run `docker push <USERNAME>/<IMAGE_REPOSITORY_NAME>:<TAG>` with the same information as the previous step. Eg. `docker push tylerlum/vrx-competitor-example:v1.2019`

9. You should be able to log onto your Dockerhub account and see your new repository. 

10. Optional: If you want to keep your repository private, you can click on your repository, then click Settings, then Make Private. To ensure that your Docker image can be evaluated, you can click Collaborators and add the desired Docker ID. Exact details about submission and Docker ID are coming soon. 

# Description

More details coming soon.