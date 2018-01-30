This tutorial will walk you through the setup required to make a computer ready to run VMRC. In order to run VMRC your computer will need a discrete graphics card and will need to satisfy the minimum [System Requirements](https://bitbucket.org/osrf/vmrc/wiki/system_requirements).

These instructions contain information for building the VMRC environment in Gazebo.

## Setup instructions ##

* Setup and install dependencies:


```
#!c++

    $ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    $ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
    $ sudo apt-get update
    $ sudo apt-get install mercurial cmake pkg-config python python-pip git gazebo7 libgazebo7-dev ros-kinetic-ros-base ros-kinetic-teleop-tools ros-kinetic-teleop-twist-keyboard ros-kinetic-gazebo-ros ros-kinetic-xacro ros-kinetic-joy libeigen3-dev wget
    $ pip install --upgrade pip
    $ pip install packaging
    $ sudo pip install appdirs
```

*  Now build a workspace for VMRC. If you are familiar with ROS catkin
workspaces, this is a similar concept. The steps to setup the workspace are:

```
    $ mkdir -p vmrc_ws/src
    $ cd vmrc_ws
```

* Download the [vmrc.repos]https://bitbucket.org/osrf/vmrc/raw/default/vmrc.repos) file into vmrc_ws:

```
wget https://bitbucket.org/osrf/vmrc/raw/default/vmrc.repos
```