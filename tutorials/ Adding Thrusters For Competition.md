#Overview
This is a script designed to generate a custom WAM-V with a set of thrusters specified from a user generated YAML file (supported thrusters can be seen in wamv_description/urdf/thrusters/, currently we have only 1 thruster type) as well as make sure that the requested thruster configuration is in compliance (as of June 06, all possible thruster configurations are in compliance).

# Quick Start Instructions:
1. launch the example world:
```roslaunch vrx_gazebo sandisland.launch```
2. look at the WAM-V in sand island. It currently has the default thruster configuration for sandisland.launch (which is H thruster configuration). You can click View=>Transparent and then see the 2 default thrusters.

3. close gazebo
4. make a directory for your custom wamv EG:
```mkdir ~/my_wamv```
```cd ~/my_wamv```

5. make a yaml file of the thrusters you want and where you want them (see example yaml thruster configuration) EG:
```gedit thruster_config.yaml```

6. run the script to generate your wamv's urdf with these newly specified thrusters:
```roslaunch vrx_gazebo generate_thrusters.launch requested:=/home/<username>/my_wamv/thruster_config.yaml xacro_target:=/home/<username>/my_wamv/my_thrusters.xacro wamv_target:=/home/<username>/my_wamv/my_wamv.urdf```
Parameters Explained:
    * requested:
this is the root path of your yaml thruster configuration file
    * xacro_target:
this is the root path of a xacro which will be generated based on your yaml file, it needs a place to live.
     * wamv_target:
this is the root path to the urdf of your wamv which will be generated
7. see the confirmation message ```wamv successfully generated``` in the terminal with no errors present
8. launch the example world with your wamv:
```roslaunch vrx_gazebo sandisland.launch urdf:=/home/<username>/my_wamv/my_wamv.urdf```
9. look at the WAM-V in Sand island. It has your thruster configuration. If everything went correctly and you used the example thruster configuration yaml file below, you can click View=>Transparent and then see the 3 desired thrusters.

10. confirm that these are the thrusters you want in the places that you want
11. observe the ros topics being published and confirm they are the topics you want:
```rostopic list```
12. close gazebo
#Description:
generate_thrusters.launch is a simple script that allows thruster configurations to be submitted by YAML (as opposed to urdf) while making sure that the thrusters are in compliance as defined by thruster_config/compliance.py.

It operates by generating macro calls specified by the user_generated yaml. It checks the number of times a macro is called for compliance ie: we could limit the number of allowed thrusters ie: all thrusters must be in a bounding box around the WAM-V. 
	
If the thruster configuration passes, the script auto fills out a xacro at xacro_target and calls a xacro command to generate the urdf at wamv_target using wamv_gazebo/urdf/wamv_gazebo.urdf.xacro.
Note:
see wamv_description/urdf/thrusters for a list of the supported xacro macros and parameters(parameters here mirror their xacro counterparts)

#Example Yaml Thruster Configuration File
```
engine:
  - prefix: "left"
    position: "-2.373776 1.027135 0.318237"
    orientation: "0.0 0.0 0.0"
  - prefix: "right"
    position: "-2.373776 -1.027135 0.318237"
    orientation: "0.0 0.0 0.0"
  - prefix: "middle"
    position: "-2.373776 0 0.318237"
    orientation: "0.0 0.0 0.0"
```