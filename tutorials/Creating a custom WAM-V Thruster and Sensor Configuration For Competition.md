# Overview
This is a script designed to generate a custom WAM-V with a set of thrusters and sensors specified from a user-generated thruster YAML file and a user-generated sensor YAML file. Supported thrusters and sensors can be seen in `wamv_description/urdf/thrusters/` and `wamv_gazebo/urdf/sensors/` (currently we have only one thruster type). This script also makes sure that the requested thruster and sensor configurations are in compliance (as of June 6, 2019, all possible thruster and sensor configurations are in compliance).

# Quick Start Instructions:
1. Launch the example world:

    ```
       $ roslaunch vrx_gazebo sandisland.launch
    ```

2. Look at the WAM-V in sand island. It currently has the default thruster configuration and sensor configuration for sandisland.launch (as of June 6, 2019, default is [H thruster configuration](https://bitbucket.org/osrf/vrx/wiki/tutorials/PropulsionConfiguration) and no sensors). In Gazebo, click View => Transparent to see the thrusters.

    ![Default_Sensors_And_Thrusters.png](https://bitbucket.org/repo/BgXLzgM/images/2154255799-Default_Sensors_And_Thrusters.png)

3. Close gazebo
4. Make a directory for your custom WAM-V, eg:
    ```
       $ mkdir ~/my_wamv
       $ cd ~/my_wamv
    ```
5. Make a yaml file of the thrusters you want and where you want them (see example yaml thruster configuration below) eg:
    ```
       $ gedit thruster_config.yaml
    ```
6. Make a yaml file of the sensors you want and where you want them (see example yaml sensor configuration below) eg:
    ```
       $ gedit sensor_config.yaml
    ```
7. Run the script to generate your WAM-V's urdf with these newly specified thrusters and sensors.
    ```
       $ roslaunch vrx_gazebo generate_wamv.launch thruster_yaml:=/home/<username>/my_wamv/thruster_config.yaml  sensor_yaml:=/home/<username>/my_wamv/sensor_config.yaml wamv_target:=/home/<username>/my_wamv/my_wamv.urdf
    ```

    Parameters Explained:

    * `thruster_yaml`: input, the full path of the thruster YAML configuration file

    * `sensor_yaml`: input, the full path of the sensor YAML configuration file

    * `wamv_target`: output, the full path to the WAM-V URDF, which will be generated

8. See the following confirmation message in the terminal with no errors present 
    ```
       WAM-V urdf file sucessfully generated. File location: <wamv_target>
    ```

9. Launch the example world with your WAM-V:
    ```
       $ roslaunch vrx_gazebo sandisland.launch urdf:=/home/<username>/my_wamv/my_wamv.urdf
    ```
10. Look at the WAM-V in sand island. It should have your thruster and sensor configurations. If everything went correctly and you used the example thruster and sensor configuration yaml files below, you should see the desired sensors and you can click View => Transparent to see the desired thrusters.

    ![Desired_Config.png](https://bitbucket.org/repo/BgXLzgM/images/2592656789-Desired_Config.png)

11. Confirm that these are the thrusters and sensors you want in the places that you want
12. Observe the ros topics being published and confirm they are the topics you want:

    ```
       $ rostopic list
    ```

13. Close gazebo

# Description:
`generate_wamv.launch` is a simple script that allows thruster and sensor configurations to be submitted by YAML (as opposed to urdf) while making sure that the thrusters and sensors are in compliance as defined by compliance.py.

It operates by generating macro calls specified by the user-generated YAML configuration files. It checks the number of times a macro is called for compliance i.e., we could limit the number of allowed thrusters/sensors ie: all thrusters/sensors must be in a bounding box around the WAM-V.   See the VRX Technical Guide for specifics on the constraints for the VRX competition, available at the [Documentation Wiki](https://bitbucket.org/osrf/vrx/wiki/documentation).
	
If the thruster/sensor configuration passes, the script creates two xacro files. One is a thruster xacro file in the same directory as the thruster yaml file with the same file name, but with a .xacro extension. The second is a sensor xacro file in the same directory as the sensor yaml file with the same file name, but with a .xacro extension. 

Next, it calls a xacro command to generate the urdf at `wamv_target` using `wamv_gazebo/urdf/wamv_gazebo.urdf.xacro`.

Note:

* See `wamv_description/urdf/thrusters` and `wamv_gazebo/urdf/sensors` for a list of the supported xacro macros and parameters (parameters here mirror their xacro counterparts)

* When calling `generate_wamv.launch`, if the `thruster_yaml` parameter is not given, then the default thruster configuration is used (as of June 6, 2019 it is H configuration)

* When calling `generate_wamv.launch`, if the `sensor_yaml` parameter is not given, then the default sensor configuration is used (as of June 6, 2019, there are no sensors by default)

* To setup a WAM-V with no thruster and no sensors, you can create empty files `empty_thruster_config.yaml` and `empty_sensor_config.yaml` and then pass them in as parameters

# Example Yaml Thruster Configuration File
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