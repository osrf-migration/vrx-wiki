# RXI Tutorial: Custom, Randomized Worlds #

Creating custom worlds will allow you to expand on the VRX simulation basics to apply the tools to your own work and to future RobotX competitions.  In this tutorial we will build an example world with an assortment of tasks that could be used in RobotX 2020.  The goal is for teams to learn how to use VRX to accelerate their development and preparation for RobotX 2020.

## Examples

* [Adding Course Elements](https://bitbucket.org/osrf/vrx/wiki/tutorials/Adding%20course%20elements)
* [Create Custom Docks](https://bitbucket.org/osrf/vrx/wiki/tutorials/CreateDocks)
* [Randomized Obstacle Course](https://bitbucket.org/osrf/vrx/pull-requests/54/obstacle-course/diff)


# Setup

For this tutorial you will create you own ROS package called `rxi` to contain the custom world definition.  This ROS package should be in the same catkin workspace as the VRX packages.  The following instructions assume that your catkin workspace is named '~/vrx_ws'.

Make a new directory for the ROS package, initialize the package by creating blank CMakelists.txt and package.xml files and build.
```
cd ~/vrx_ws/src/
mkdir rxi
catkin_create_pkg rxi
cd ~/vrx_ws
catkin_make
```

Make ROS aware of the new package by
```
source ~/vrx_ws/devel/setup.bash
```

Optional: This is a good time to setup version control for your `rxi` package.  If you are familiar with version control (e.g., git

## Create working launch and world files

Make new directories in your rxi package and copy exising launch and world files from the `vrx_gazebo` package to provide a working example.
```
cd ~/vrx_ws/src/rxi
mkdir launch
mkdir worlds
cd launch
roscp vrx_gazebo vrx.launch ./rxi.launch
cd ../worlds/
roscp vrx_gazebo sandisland.world.xacro rxi.world.xacro
```

Now we will edit our new files so that we can verify they are a working starting point for creating a custom environment.

The `rxi.world.xacro` file uses the [xacro](http://wiki.ros.org/xacro) (XML Macros) utility to generate a [Gazebo world](http://gazebosim.org/tutorials?tut=build_world) file.  The file currently creates the basic Sand Island environment, an ocean wave model and an ocean wind model.  In order to setup the package to process the xacro file we'll want to do the following within the `rxi` package:

1. Edit the `package.xml` file and add the following line:
```
<depend>xacro</depend>
```
2. Edit the `CMakeLists.txt` file and add the following lines at the bottome of the file:
```
find_package(catkin REQUIRED COMPONENTS
  xacro)

catkin_package(
  CATKIN_DEPENDS xacro)

# Generate world files from xacro and install
xacro_add_files(
  worlds/rxi.world.xacro
  INORDER INSTALL DESTINATION worlds)

# Install all the world files
install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds)
```
3. Re-make with
```
catkin_make
```

Now when you build package the `rxi.world.xacro` file is processed, generating the `rxi.world` file that is installed in the system. (To see where try  `find ~/vrx_ws -name "rxi.world"`).

Next we will edit the `rxi.launch` file so that it uses the generated `rxi.world` file.

* Edit `rxi.launch` and change the default value of the world argument to point to the new `rxi.world` file in the `rxi` ROS package.

Now you should be able to call your new launch file with...
```
source devel/setup.bash 
roslaunch rxi rxi.launch
```

![rxi example](./images/rxilaunch.png)

