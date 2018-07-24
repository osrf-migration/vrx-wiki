This tutorial will walk you through the setup required to make a computer ready to run the VMRC simulations. In order to run VMRC your computer will need a discrete graphics card and will need to satisfy the minimum [System Requirements](https://bitbucket.org/osrf/vmrc/wiki/system_requirements).

These instructions contain information for building the VMRC environment in Gazebo.

# Option 1: Install all software in your host system #

* Setup and install dependencies:


```
#!bash

    $ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    $ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
    $ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
    $ wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
    $ sudo apt-get update
    $ sudo apt-get install mercurial cmake pkg-config python git gazebo7 libgazebo7-dev ros-kinetic-ros-base ros-kinetic-teleop-tools ros-kinetic-teleop-twist-keyboard ros-kinetic-gazebo-plugins ros-kinetic-gazebo-ros ros-kinetic-hector-gazebo-plugins ros-kinetic-xacro ros-kinetic-joy libeigen3-dev wget
```

* Now build a workspace for VMRC. If you are familiar with ROS catkin
workspaces, this is a similar concept. The steps to setup the workspace are:

```
#!bash

    $ mkdir -p ~/vmrc_ws/src
    $ cd ~/vmrc_ws/src
```

* Clone the VMRC repository:

```
#!bash
    $ hg clone https://bitbucket.org/osrf/vmrc
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