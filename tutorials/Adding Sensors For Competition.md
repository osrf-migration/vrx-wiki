#Overview
This is a script designed to generate a custom WAM-V with a set of sensors specified from a user generated YAML file (supported sensors can be seen here https://bitbucket.org/osrf/vrx/src/default/wamv_gazebo/urdf/sensors/) as well as make sure that the requested sensor configuration is in compliance (as of June 03, all possible sensor configurations are in compliance).

# Quick Start Instructions:
1. launch the example world:
```roslaunch vrx_gazebo sandisland.launch```
2. look at the WAM-V in sand island. It currently has the default sensor configuration for sandisland.launch (which is none). You should see something like this:
![WAMV0.png](https://bitbucket.org/repo/BgXLzgM/images/112186942-WAMV0.png)

3. close gazebo
4. make a directory for your custom wamv EG:
```mkdir ~/my_wamv```
```cd ~/my_wamv```

5. make a yaml file of the sensors you want and where you want them (see example yaml sensor configuration) EG:
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
7. see the confirmation message ```wamv successfully generated``` in the terminal with no errors present
8. launch the example world with your wamv:
```roslaunch vrx_gazebo sandisland.launch urdf:=/home/<username>/my_wamv/my_wamv.urdf```
9. look at the WAM-V in Sand island. It has your sensor configuratio. If everything went correctly and you used the example sensor configuration yaml file below, you should see something like this:
![WAMV1.png](https://bitbucket.org/repo/BgXLzgM/images/3232049032-WAMV1.png)
10. confirm that these are the sensors you want in the places that you want
11. observe the ros topics being published and confirm they are the topics you want:
```rostopic list```
12. close gazebo
#Description:
generate_sensors.launch is a simple script that allows sensor configurations to be submitted by YAML (as opposed to urdf) while making sure that the sensors are in compliance as defined by sensor_config/compliance.py and are allowed (included by wamv_gazebo/urdf/wamv_gazebo.urdf.xacro).

It operates by generateing macro calls specified by the user_generated yaml. It checks the number of times a macro is called for compliance ie: only one lidar allowed as well as the parameters of each macro call for compliance ie: all sensors must be in a bounding box around the WAM-V. Sensors not included by wamv_gazebo/urdf/wamv_gazebo.urdf.xacro are not  not supported and will generate an XML error if attempted to be used.
	
If the sensor configuration passes, the script auto fills out a xacro at xacro_target and calls a xacro command to generate the urdf at wamv_target using wamv_gazebo/urdf/wamv_gazebo.urdf.xacro.
Note:
see wamv_gazebo/urdf/sensors for a list of the supported xacro macros and parameters(parameters here mirror thier xacro counterparts)

#Example Yaml Sensor Configuration File
```
wamv_camera:
    - name: front_camera
      x: 0.75
      y: 0.3
      z: 2
      P: ${radians(15)}
    - name: front_left_camera
      x: 0.75
      y: 0.1
      P: ${radians(15)}
    - name: front_right_camera
      x: 0.75
      y: -0.1
      P: ${radians(15)}
    - name: middle_right_camera
      x: 0.75
      y: 0.3
      P: ${radians(15)}
wamv_gps:
    - name: gps_wamv
      x: -0.85
wamv_imu:
    - name: imu_wamv
      y: -0.2
wamv_p3d:
    - name: p3d_wamv
wamv_3d_lidar:
    - name: lidar_wamv
      P: ${radians(8)}
```