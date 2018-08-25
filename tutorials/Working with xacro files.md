# What is xacro?
[Xacro](http://wiki.ros.org/xacro) is like a programmable xml file, allowing you to include other files, define functions (called macros), store variables, etc. The VMRC packages make heavy use of xacro to reduce code reuse and make composing new models/worlds easy.

## Generating XML from xacro
Gazebo and most other programs which are configured with XML cannot parse xacro files directly. Instead, you must "compile" your xacro files into static XML. There are several ways you can do this:
### From Terminal
To generate ```my_wamv.urdf``` from ```my_wamv.urdf.xacro```, run this command:

```$ rosrun xacro xacro --inorder my_wamv.urdf.xacro -o my_wamv.urdf```
### In CMakeLists.txt
A convenient option is to add your xacro files as build targets, so XML will be generated whenever you build your project with ```catkin_make```. This has the added benefit of failing your build if the xacro is invalid, allowing you to find this issue at build time, not runtime.

To have catkin generate xml from xacro files, first add the xacro library to your find_package line in your project's CMakeLists.txt:
```
find_package(catkin REQUIRED COMPONENTS
  xacro
  ...
)
```
Then add your files as targets:
```
xacro_add_files(
  worlds/my_example_world.world.xacro
  INORDER INSTALL DESTINATION worlds
)
```
This will generate ```devel/share/worlds/your_project_name/my_example_world.world``` in your workspace.

### In launch file
Launch files can also parse xacro and pass the compiled XML along to nodes at runtime. Here's an example:
```
<?xml version="1.0"?>
<launch>
  <arg name="urdf" default="$(find my_package)/urdf/wamv_gazebo.urdf"/>
  <param name="robot_description" command="$(find xacro)/xacro --inorder $(arg urdf)"/>
</launch>
```
when this is run, the ```robot_description``` ROS parameter will be set to the compiled xacro file in ```my_package/urdf/wamv_gazebo.urdf```.

## Using Macros
The VMRC packages contain many xacro macros for quickly composing XML files. You can read more about macros [here](http://wiki.ros.org/xacro#Macros). 

Here is an example macro included to create a gazebo camera sensor with the ROS plugin:
```
$ roscat wamv_gazebo wamv_camera.xacro 
```
```
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro">
  <xacro:macro name="wamv_camera" params="name x:=0.5 y:=0 z:=1.5 R:=0 P:=0 Y:=0">
    <link name="${name}_link"/>
    <joint name="${name}_joint" type="fixed">
      <origin xyz="${x} ${y} ${z}" rpy="${R} ${P} ${Y}"/>
      <parent link="base_link"/>
      <child link="${name}_link"/>
    </joint>       
    ...
  </xacro:macro>
</robot>
```
Notice the parameters *name, x, y, z, R, P ,Y* which are refereed to as variables within the macro. Some of them have defaults so may be excluded when the macro is used.

Here is an example URDF xacro which uses this macro:
```
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro"
       name="WAM-V">
  ...
  <!-- Include the macro -->
  <xacro:include filename="$(find wamv_gazebo)/urdf/sensors/wamv_camera.xacro" />
  <!-- Call the macro to add a front camera -->
  <xacro:wamv_camera name="front_camera"  x="5" Y="1.57" />
  ...
</robot>
```