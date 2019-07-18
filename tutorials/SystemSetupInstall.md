This tutorial will walk you through the setup required to make a computer ready to run the VRX simulations. In order to run VRX your computer will need a discrete graphics card and will need to satisfy the minimum [System Requirements](https://bitbucket.org/osrf/vrx/wiki/system_requirements).

These instructions contain information for preparing your system environment.

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
    $ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
    $ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
    $ wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
    $ sudo apt update
    $ DIST=melodic
    $ GAZ=gazebo9
    $ sudo apt install cmake mercurial git ruby libeigen3-dev ${GAZ} lib${GAZ}-dev pkg-config python ros-${DIST}-gazebo-plugins ros-${DIST}-gazebo-ros ros-${DIST}-hector-gazebo-plugins ros-${DIST}-joy ros-${DIST}-joy-teleop ros-${DIST}-key-teleop ros-${DIST}-robot-localization ros-${DIST}-robot-state-publisher ros-${DIST}-rviz ros-${DIST}-ros-base ros-${DIST}-teleop-tools ros-${DIST}-teleop-twist-keyboard ros-${DIST}-velodyne-simulator ros-${DIST}-xacro ros-${DIST}-rqt ros-${DIST}-rqt-common-plugins protobuf-compiler
```

# Option 1: Install our pre-built package

```
#!bash

    $ sudo apt install ros-melodic-vrx-gazebo
```

## To test:

* Source the ROS `setup.bash` file*****:

```
#!bash
    $ source /opt/ros/melodic/setup.bash
```

* Launch the vrx simulation with a simple world:

```
#!bash
    $ roslaunch vrx_gazebo sandisland.launch
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

## To test:

* Source the VRX `setup.bash` file*****:

```
#!bash
    $ source  ~/vrx_ws/devel/setup.bash
```

* Launch the vrx simulation with a simple world:

```
#!bash
    $ roslaunch vrx_gazebo sandisland.launch
```

*****Remember to run this command every time you open a new terminal.