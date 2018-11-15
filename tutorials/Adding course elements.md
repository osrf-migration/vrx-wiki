# Overview
The vmrc_gazebo package has a basic robotx course with water, sky, coastline, and some of the challenges in fixed positions. It is easy to create your own static course or even add new challenges into a running simulation.

## Prerequisites
This guide assumes you already have installed ROS and the vmrc packages as covered in the [setup instructions](https://bitbucket.org/osrf/vmrc/wiki/tutorials/SystemSetupInstall).

***
## Creating a world file
A world file defines the initial environment gazebo starts in, including lighting, sky, ground, and models. Let's copy one of the examples from vmrc_gazebo as a starting point for our world file:
```
$ mkdir example_vmrc_package
$ cd example_vmrc_package/
$ roscp vmrc_gazebo example_course.world.xacro .
```
Now you should have a file that looks something like this:
```
cat example_course.world.xacro
```
```
<?xml version="1.0" ?>
<!-- World containing sandisland model and some course challenges -->
<sdf version="1.4" xmlns:xacro="http://ros.org/wiki/xacro">
  <world name="robotx_example_course">
    <xacro:include filename="$(find vmrc_gazebo)/worlds/macros.xacro" />
    <xacro:sandisland />
    <include>
      <uri>model://robotx_navigation_challenge</uri>
      <pose>15 0 2 0 0 0</pose>
    </include>
    <include>
      <uri>model://robotx_light_buoy</uri>
      <pose>60 0 0 0 0 0</pose>
    </include>
  </world>
</sdf>
```
Notice that this is an **.xacro** file. If you aren't familiar with xacro files, you should read [this](https://bitbucket.org/osrf/vmrc/wiki/tutorials/Working%20with%20xacro%20files) tutorial first.

Let's go through this file and make some changes. First, notice the first two lines within the **<world>** tag:
```
<xacro:include filename="$(find vmrc_gazebo)/worlds/macros.xacro" />
<xacro:sandisland />
```
The first line imports xacro macros defined in vmrc_gazebo, including the one used on the next line. The second line calls the sandisland macro, which sets up an empty sand island environment (only water, sky, and coastline). You will likely want to include these lines in your world files unless you are simulating a location other than sandisland.

The remainder of the file is simply adding different challenges into the course. You can see a list of models / challenges included in vmrc_gazebo [here](https://bitbucket.org/osrf/vmrc/src/default/vmrc_gazebo/README.md).

Let's add a few buoys into the world to make some obstacles to avoid. Put these lines after the other **<include>** tags:
```
    <include>
      <!-- Small buoy found in vmrc_gazebo -->
      <uri>model://polyform_a3</uri>
      <!-- X Y Z roll pitch yaw, relative to center of gazebo world (a point out in the water) -->
      <pose>-10 2 0 0 0 0</pose>
    </include>
    <include>
      <uri>model://polyform_a7</uri>
      <pose>-15 8 0 0 0 0</pose>
    </include>
    <include>
      <uri>model://surmark950400</uri>
      <pose>-12.5 -3 0 0 0 0</pose>
    </include>
```

### Running vmrc_gazebo with a custom world
Now that you have created a new world file, you can launch the simulation again with this world:

First, generate the compiled XML from the xacro file using this or [another method](https://bitbucket.org/osrf/vmrc/wiki/tutorials/Working%20with%20xacro%20files):
```
rosrun xacro xacro --inorder example_course.world.xacro > my_world.world
```
Next, run the simulation with a custom world argument:
```
roslaunch vmrc_gazebo sandisland.launch world:=`pwd`/my_world.world
```
If everything went well, you should see 3 new objects added into Gazebo:

![customobjects.png](https://bitbucket.org/repo/BgXLzgM/images/3874067301-customobjects.png)