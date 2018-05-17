This tutorial will walk you through the setup required to make a computer ready to run the VMRC simulations. In order to run VMRC your computer will need a discrete graphics card and will need to satisfy the minimum [System Requirements](https://bitbucket.org/osrf/vmrc/wiki/system_requirements).

These instructions contain information for building the VMRC environment in Gazebo.

## Setup instructions ##

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