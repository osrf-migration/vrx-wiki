This tutorial will walk you through the setup required to make a computer ready to run the VRX simulations. In order to run VRX your computer will need a discrete graphics card and will need to satisfy the minimum [System Requirements](https://bitbucket.org/osrf/vrx/wiki/system_requirements).

These instructions contain information for building the VRX environment in Gazebo.

# Install all prerequisites in your host system #

* Because the simulation uses some relatively new (as of winter 2019) features in ROS and Gazebo, it is highly recommended that you upgrade the packages installed on your system:

```
#!bash

    $ sudo apt update
    $ sudo apt full-upgrade
```


* Setup and install dependencies:


```
#!bash

    $ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    $ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
    $ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
    $ wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
    $ sudo apt update
    $ sudo apt install cmake mercurial gazebo9 git libeigen3-dev libgazebo9-dev pkg-config python ros-melodic-gazebo-plugins ros-melodic-gazebo-ros ros-melodic-hector-gazebo-plugins ros-melodic-joy ros-melodic-joy-teleop ros-melodic-robot-localization ros-melodic-ros-base ros-melodic-teleop-tools ros-melodic-teleop-twist-keyboard ros-melodic-velodyne-simulator ros-melodic-xacro ruby wget
```

# Option 1: Run our pre-built package

```
#!bash

    $ sudo apt install ros-melodic-vrx-gazebo
```

# Option 2: Build VRX from source

* Now build a workspace for VRX. If you are familiar with ROS catkin
workspaces, this is a similar concept. The steps to setup the workspace are:

```
#!bash

    $ mkdir -p ~/vrx_ws/src
    $ cd ~/vrx_ws/src
```

* Clone the VRX repository:

```
#!bash
    $ hg clone https://bitbucket.org/osrf/vrx
```

## Build instructions ##

* Source the ROS `setup.bash` file:

```
#!bash

    $ source /opt/ros/melodic/setup.bash
```

* Build all the software:

```
#!bash
    $ cd ~/vrx_ws
    $ catkin_make
```

# Test Run

* Source the ROS `setup.bash` file:

```
#!bash
    $ source /opt/ros/melodic/setup.bash
```

* **Only needed if you built from source:**


```
#!bash
    $ source  ~/vrx_ws/devel/setup.bash
```

* Launch the vrx simulation:

```
#!bash
    $ roslaunch vrx_gazebo sandisland.launch
```