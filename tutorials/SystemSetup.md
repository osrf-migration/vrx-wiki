This tutorial will walk you through the setup required to make a computer ready to run the VMRC simulations. In order to run VMRC your computer will need a discrete graphics card and will need to satisfy the minimum [System Requirements](https://bitbucket.org/osrf/vmrc/wiki/system_requirements).

These instructions contain information for building the VMRC environment in Gazebo.

# Option 1: Install all software in your host system #

* Setup and install dependencies:


```
#!bash

    $ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    $ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
    $ sudo apt-get update
    $ sudo apt-get install mercurial cmake pkg-config python python-pip git gazebo7 libgazebo7-dev ros-kinetic-ros-base ros-kinetic-teleop-tools ros-kinetic-teleop-twist-keyboard ros-kinetic-gazebo-plugins ros-kinetic-gazebo-ros ros-kinetic-hector-gazebo-plugins ros-kinetic-xacro ros-kinetic-joy libeigen3-dev wget
    $ pip install --upgrade pip
    $ pip install packaging
    $ pip install vcstools
    $ sudo pip install appdirs
```

Note - when upgrading the ```pip``` package, you may get a message such as...
```
You are using pip version 8.1.1, however version 9.0.1 is available.
You should consider upgrading via the 'pip install --upgrade pip' command.
```
This is not a problem, version 8.1.1 is sufficient.

* Download the [vmrc.repos](https://bitbucket.org/osrf/vmrc/raw/default/vmrc.repos) file into `vmrc_ws`:

```
#!bash
    $ mkdir -p ~/workspace && cd ~/workspace
    $ wget https://bitbucket.org/osrf/vmrc/raw/default/vmrc.repos
```

* Import the repositories from vmrc.repos:

```
#!bash

    $ vcs import . < vmrc.repos
```

* Now build a workspace for VMRC. If you are familiar with ROS catkin
workspaces, this is a similar concept. The steps to setup the workspace are:

```
#!bash

    $ mkdir -p ~/vmrc_ws/src
    $ cd ~/vmrc_ws
    $ catkin_init_workspace src
```

* Create symbolic links to the ROS packages that we're using:

```
#!bash
    $ cd ~/vmrc_ws/src
    $ ln -s ~/workspace/robotx_gazebo
    $ ln -s ~/workspace/wamv_description
```

## Build instructions ##

* Source the ROS `setup.bash` file:

```
#!bash

    $ source /opt/ros/kinetic/setup.bash
```

* Build all the software:

```
#!bash
    $ cd ~/vmrc_ws
    $ catkin_make
```

## Test Run ##

```
#!bash
    $ cd ~/vmrc_ws
    $ source devel/setup.bash
    $ roslaunch robotx_gazebo sandisland.launch 
```

# Option 2: Install all software using Docker #

It's possible to use [Docker](https://www.docker.com/) to simplify the installation process or if you prefer to leave your host system untouched. We have created a Docker image that you'll need to build following these instructions.

## Install Docker ##

Docker has two available versions: Community Edition (CE) and Enterprise Edition (EE). In this tutorial, we'll install the CE version.

* Remove old versions of Docker (if installed):

```
#!bash
    $ sudo apt-get remove docker docker-engine docker.io
```

* Install the following dependencies needed to setup an external package repository:

```
#!bash
   $ sudo apt-get install apt-transport-https ca-certificates curl software-properties-common
```

* Add the official GPG key of Docker:

```
#!bash
    $ curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
```

* Add the stable package repository:

```
#!bash
    $ sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
```

**Note for Ubuntu Bionic (18.04 LTS) users only **: There's no stable package yet. Instead, run the following command:

```
#!bash
    $ sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) edge"
```

* Now, we can install Docker:

```
#!bash
    $ sudo apt-get update && sudo apt-get install docker-ce
```
For Ubuntu 16.04, docker-ce should be roughly version 18.03.1 (as of June, 2018).


* Check your Docker installation:

```
#!bash
    $ sudo docker run hello-world
```

You should see the message `Hello from Docker!` confirming that your installation was successfully completed.

## Install Nvidia Docker ##

* Remove old versions of [Nvidia Docker](https://github.com/NVIDIA/nvidia-docker):

```
#!bash
    $ docker volume ls -q -f driver=nvidia-docker | xargs -r -I{} -n1 docker ps -q -a -f volume={} | xargs -r docker rm -f
    $ sudo apt-get purge -y nvidia-docker
```

* Setup the Nvidia Docker repository. Choose only one block based on your
```
#!bash
    $ curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
    $ distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
    $ curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
        sudo tee /etc/apt/sources.list.d/nvidia-docker.list
    $ sudo apt-get update
```

* Next, install Nvidia Docker (version 2):

```
#!bash
    $ sudo apt-get install -y nvidia-docker2
    $ sudo pkill -SIGHUP dockerd
```

* Verify the installation:

```
#!bash
    $ docker run --runtime=nvidia --rm nvidia/cuda nvidia-smi
```

This command should print your GPU information, for example...
![Screenshot from 2018-06-20 08-21-43.png](https://bitbucket.org/repo/BgXLzgM/images/403079041-Screenshot%20from%202018-06-20%2008-21-43.png)


## Build the VMRC image ##

* Download the Docker VMRC image and some scripts:

```
#!bash
    $ mkdir -p ~/vmrc_docker/vmrc && cd ~/vmrc_docker/vmrc
    $ wget https://bitbucket.org/osrf/vmrc/raw/docker/docker/vmrc/Dockerfile
    $ cd ..
    $ wget https://bitbucket.org/osrf/vmrc/raw/docker/docker/build.bash
    $ wget https://bitbucket.org/osrf/vmrc/raw/docker/docker/run.bash
    $ chmod u+x build.bash run.bash
```

* Build your VMRC Docker image:

```
#!bash
    $ ./build.bash vmrc
```

* Run your VMRC container:

```
#!bash
    $ ./run.bash vmrc
```

* Test your VMRC installation:

```
#!bash
    $ roslaunch robotx_gazebo sandisland.launch
```