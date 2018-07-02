# What is xacro?
[Xacro](http://wiki.ros.org/xacro) is like a programmable xml file, allowing you to include other files, define functions (called macros), store variables, etc. The VMRC packages make heavy use of xacro to reduce code reuse and make composing new models/worlds easy.

## Generating XML from xacro
Gazebo and most other programs which are configured with XML cannot parse xacro files directly. Instead, you must "compile" your xacro files into static XML. There are several ways you can do this:
### From Terminal
To generate ```my_robot.urdf``` from ```my_robot.urdf.xacro```, run this command:

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
Launch files can also parse xacro and the compiled XML along to nodes at runtime. Here's an example:
```
<?xml version="1.0"?>
<launch>
  <arg name="urdf" default="$(find my_package)/urdf/wamv_gazebo.urdf"/>
  <param name="robot_description" command="$(find xacro)/xacro --inorder $(arg urdf)"/>
</launch>
```
when this is run, the ```robot_description``` ROS parameter will be set to the compiled xacro file in ```my_package/urdf/wamv_gazebo.urdf```.