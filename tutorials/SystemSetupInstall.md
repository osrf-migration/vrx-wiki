This tutorial will walk you through the setup required to make a computer ready to run the VMRC simulations. In order to run VMRC your computer will need a discrete graphics card and will need to satisfy the minimum [System Requirements](https://bitbucket.org/osrf/vmrc/wiki/system_requirements).

These instructions contain information for building the VMRC environment in Gazebo.

# Install all prerequisites in your host system #

* Because the simulation uses some relatively new (as of summer 2018) features in ROS and Gazebo, it is highly recommended that you upgrade the packages installed on your system:

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
    $ sudo apt install cmake mercurial gazebo7 git libeigen3-dev libgazebo7-dev pkg-config python ros-kinetic-gazebo-plugins ros-kinetic-gazebo-ros ros-kinetic-hector-gazebo-plugins ros-kinetic-joy ros-kinetic-joy-teleop ros-kinetic-robot-localization ros-kinetic-ros-base ros-kinetic-teleop-tools ros-kinetic-teleop-twist-keyboard ros-kinetic-velodyne-simulator ros-kinetic-xacro ruby wget
```

# Option 1: Run our pre-built package

```
#!bash

    $ sudo apt install ros-kinetic-vmrc-gazebo
```

# Option 2: Build VMRC from source

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

# Test Run

* Source the ROS `setup.bash` file:

```
#!bash
    $ source /opt/ros/kinetic/setup.bash
```

* **Only needed if you built from source:**


```
#!bash
    $ source  ~/vmrc_ws/devel/setup.bash
```

* Launch the VMRC simulation:

```
#!bash
    $ roslaunch vmrc_gazebo sandisland.launch 
```