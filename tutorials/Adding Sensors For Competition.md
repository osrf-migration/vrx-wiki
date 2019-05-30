## Quick Start Instructions:
1. launch the example world:
```roslaunch vrx_gazebo sandisland.launch```
2. look at the WAM-V in sand island. It currently has the default sensor configuration.
3. close gazebo
4. make a directory for your custom wamv EG:
```mkdir ~/my_wamv```
```cd ~/my_wamv```

5. make a yaml file of the sensors you want and where you want them (see example yaml sensor configuration file) EG:
```gedit sensor_config.yaml```

6. run the script to generate your wamv's urdf with these newly specified sensors:
```roslaunch vrx_gazebo generate_sensors.launch requested:=/home/<username>/my_wamv/sensor_config.yaml xacro_target:=/home/<username>/my_wamv/my_sensors.xacro wamv_target:=/home/<username>/my_wamv/my_wamv.urdf```
Parameters Explained:
    * requested:
this is the root path of your yaml sensor configuration file
    * xacro_target:
this is the root path of a xacro which will be generated based on your yaml file, it needs a place to live.
     * wamv_target:
this is the root path to the urdf of your wamv which will be generated
7. see the confirmation message ```wamv sucessfully generated``` in the terminal and close the script
8. launch the example world with your wamv:
```roslaunch vrx_gazebo sandisland.launch urdf:=/home/<username>/my_wamv/my_wamv.urdf```
9. look at the WAM-V in Sand island. It has your sensor configuration
10. confirm that these are the sensors you want in the places that you want
11. observe the ros topics being published and confirm they are the topics you want:
```rostopic list```
12. close gazebo:
#Description:
generate_sensors is a simple script that allows sensor configurations to be submitted by YAML (instead of urdf) while making sure that the sensors are in compliance as defined by sensor_config/compliance.py and are
allowed (defined in wamv_gazebo/urdf/sensors).

It operates by looking at all macros defined in a directory(in this case, wamv_gazebo/urdf/sensors)
and all macros called in sensor_config.yaml (as well as their parameters). It makes sure that all the macros
called are avalible in wamv_gazebo/urdf/sensors as well as the specified parameters. It also checks the number
of times a macro is called for compliance ie: only ONE lidar allowed. It also checks the parameters of each
macro call for compliance ie: all sensors must be in a bounding box around the WAM-V.
	
If the sensor configuration passes, then the script auto fills out xacro_target, calls: rosrun xacro xacro to generate the urdf at wamv_target using wamv_gazebo/urdf/wamv_gazebo.urdf.xacro and the sensors xacro at xacro_target