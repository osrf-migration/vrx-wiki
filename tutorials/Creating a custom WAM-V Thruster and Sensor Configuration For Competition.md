# Overview
The purpose of this tutorial is to demonstrate how to create a custom WAM-V Thruster and Sensor Configuration for Competition. This involves writing a user-generated thruster YAML file and a user-generated sensor YAML file, and then running a script that will generate a custom WAM-V URDF file with the specified thrusters and sensors. This WAM-V URDF file can then be passed in as a parameter to the VRX simulation `roslaunch` files.

In addition, this script also makes sure that the requested thruster and sensor configurations are in compliance  with basic VRX constraints. More details about compliance below.

# Quick Start Instructions:

## Example 1: How to use generate_wamv.launch

1. First be sure your VRX workspace is built and sourced. Then launch the example world:

    ```
       $ roslaunch vrx_gazebo sandisland.launch
    ```

2. Look at the WAM-V in sand island. It currently has the default thruster configuration and sensor configuration for `sandisland.launch` (as of June 6, 2019, the default configuration is the [H thruster configuration](https://bitbucket.org/osrf/vrx/wiki/tutorials/PropulsionConfiguration) and no sensors). In Gazebo, click View => Transparent to see the thrusters.

    ![Default_Sensors_And_Thrusters.jpg](https://bitbucket.org/repo/BgXLzgM/images/1178967311-Default_Sensors_And_Thrusters.jpg)

3. Close gazebo
4. Make a directory for your custom WAM-V, eg:

    ```
       $ mkdir ~/my_wamv
    ```

5. Scroll down this tutorial and copy the contents of **Example Compliant Yaml Thruster Configuration File 1**. Next, paste those contents into a new file.

    ```
       $ gedit ~/my_wamv/thruster_config.yaml
    ```

    Later, we will edit this file to customize your WAM-V.

6. Scroll down this tutorial and copy the contents of **Example Compliant Yaml Sensor Configuration File 1**. Next, paste those contents into a new file.

    ```
       $ gedit ~/my_wamv/sensor_config.yaml
    ```

    Later, we will edit this file to customize your WAM-V.

7. Run the script to generate your WAM-V's URDF with these newly specified thrusters and sensors.

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

    ![Desired_Thruster_Config.jpg](https://bitbucket.org/repo/BgXLzgM/images/3459109275-Desired_Thruster_Config.jpg)

    ![Desired_Sensors.jpg](https://bitbucket.org/repo/BgXLzgM/images/1664943221-Desired_Sensors.jpg)

11. Confirm that these are the thrusters and sensors you want in the places that you want
12. Observe the ros topics being published and confirm they are the topics you want:

    ```
       $ rostopic list
    ```

13. Close gazebo

## Example 2: How to Further Customize Your WAM-V

Next, let's customize the WAM-V further.

1. Scroll down this tutorial and copy the contents of **Example Compliant Yaml Thruster Configuration File 2**. Next, paste those contents into the `thruster_config.yaml` file. Note the addition of the third thruster.

    ```
       $ gedit ~/my_wamv/thruster_config.yaml
    ```

2. Scroll down this tutorial and copy the contents of **Example Compliant Yaml Sensor Configuration File 2**. Next, paste those contents into the `sensor_config.yaml` file. Note the removal of the cameras.

    ```
       $ gedit ~/my_wamv/sensor_config.yaml
    ```

3. Next, we need to run `generate_wamv.launch` again to use these new yamls files to create a new urdf file.

    ```
       $ roslaunch vrx_gazebo generate_wamv.launch thruster_yaml:=/home/<username>/my_wamv/thruster_config.yaml  sensor_yaml:=/home/<username>/my_wamv/sensor_config.yaml wamv_target:=/home/<username>/my_wamv/my_wamv_2.urdf
    ```

4. See the following confirmation message in the terminal with no errors present 

    ```
       WAM-V urdf file sucessfully generated. File location: <wamv_target>
    ```

5. Launch the example world with your WAM-V:

    ```
       $ roslaunch vrx_gazebo sandisland.launch urdf:=/home/<username>/my_wamv/my_wamv_2.urdf
    ```

6. Look at the WAM-V in sand island. It should have your new thruster and sensor configurations. If everything went correctly and you used the example thruster and sensor configuration yaml files below, you should see the desired sensors and you can click View => Transparent to see the desired thrusters. Please note the missing cameras and additional thruster.

![Desired_Configuration_2.jpg](https://bitbucket.org/repo/BgXLzgM/images/442187015-Desired_Configuration_2.jpg)

## Example 3: Non-compliance
Next, let's see how compliance works.

1. Scroll down this tutorial and copy the contents of **Example Non-compliant Yaml Thruster Configuration File**. Next, paste those contents into the `thruster_config.yaml` file. Note the addition of the third thruster in a non-compliant position.

    ```
       $ gedit ~/my_wamv/thruster_config.yaml
    ```


2. Scroll down this tutorial and copy the contents of **Example Non-compliant Yaml Sensor Configuration File**. Next, paste those contents into the `sensor_config.yaml` file. Note the addition of 5 cameras.

    ```
       $ gedit ~/my_wamv/sensor_config.yaml
    ```

3. Next, we need to run `generate_wamv.launch` again to use these new yamls files to create a new urdf file.

    ```
       $ roslaunch vrx_gazebo generate_wamv.launch thruster_yaml:=/home/<username>/my_wamv/thruster_config.yaml  sensor_yaml:=/home/<username>/my_wamv/sensor_config.yaml wamv_target:=/home/<username>/my_wamv/my_wamv_3.urdf
    ```

4. See the following messages in the terminal

```
/home/tylerlum/vrx_ws/src/vrx/vrx_gazebo/src/vrx_gazebo_python/generator_scripts/wamv_config/thruster_compliance/bounding_boxes.yaml
[ERROR] [1562799557.237759]: engine second_right is out of bounds
[ERROR] [1562799557.239173]: engine second_right is at xyz=(-2.373776, -1.027135, 0.318237), it must fit in at least one of the following boxes:
[ERROR] [1562799557.240699]:   <Box name:thruster_compliance_port_aft x:[-1.75, -2.75] y:[1.5,0.5] z:[0.6,-0.6] remaining_space:0>
[ERROR] [1562799557.242496]:   <Box name:thruster_compliance_star_for x:[1.5, 0.5] y:[-0.5,-1.5] z:[0.6,-0.6] remaining_space:1>
[ERROR] [1562799557.244069]:   <Box name:thruster_compliance_port_for x:[1.5, 0.5] y:[1.5,0.5] z:[0.6,-0.6] remaining_space:1>
[ERROR] [1562799557.245253]:   <Box name:thruster_compliance_star_aft x:[-1.75, -2.75] y:[-0.5,-1.5] z:[0.6,-0.6] remaining_space:0>
[ERROR] [1562799557.246448]:   <Box name:thruster_compliance_middle x:[1.5, -1.0] y:[0.5,-0.5] z:[0.6,-0.6] remaining_space:1>
/home/tylerlum/vrx_ws/src/vrx/vrx_gazebo/src/vrx_gazebo_python/generator_scripts/wamv_config/sensor_compliance/bounding_boxes.yaml
[ERROR] [1562799557.271779]: Too many wamv_camera requested
[ERROR] [1562799557.272903]:   maximum of 3 wamv_camera allowed
xacro: in-order processing became default in ROS Melodic. You can drop the option.

WAM-V urdf file sucessfully generated. File location: /home/tylerlum/my_wamv/my_wamv_3.urdf
```

The URDF file is still created, but these error messages show why your configuration is not compliant. Next, launch the example world with your WAM-V:

```
$ roslaunch vrx_gazebo sandisland.launch urdf:=/home/<username>/my_wamv/my_wamv_3.urdf
```

Look at the WAM-V in sand island. It should have your new thruster and sensor configurations. If everything went correctly and you used the example thruster and sensor configuration yaml files below, you should see the desired sensors and you can click View => Transparent to see the desired thrusters. Please note the missing cameras and additional thruster.

![Five Cameras.png](https://bitbucket.org/repo/BgXLzgM/images/4030317114-Five%20Cameras.png)

# Compliance

As of July 10, 2019 there are 3 main compliance rules

* All sensors must be contained within one of the sensor bounding boxes. All thrusters must be contained within one of the thruster bounding boxes. The details of the bounding boxes can be found [here](TODO add link once merged). The image below shows the position of the bounding boxes. 
TODO: add image of bounding boxes

* The number of each sensor and thruster in the configuration must be within the limit defined [here for sensors](https://bitbucket.org/osrf/vrx/src/default/vrx_gazebo/src/vrx_gazebo_python/generator_scripts/wamv_config/sensor_compliance/numeric.yaml) and [here for thrusters](https://bitbucket.org/osrf/vrx/src/default/vrx_gazebo/src/vrx_gazebo_python/generator_scripts/wamv_config/thruster_compliance/numeric.yaml). 

* For thrusters, there can only be one thruster in each bounding box. This is to prevent teams from stacking thrusters together in one location, which is physically infeasible.

Please note that if you call `generate_wamv.launch` on non-compliant configuration YAML files, red error messages will be printed but the URDF file will still be created and be able to be used as usual. However, it is not a valid configuration for the VRX competition. 

# Details of Implementation
`generate_wamv.launch` is a simple script that takes in thruster and sensor configurations YAML files as inputs and outputs a urdf file to be used in simulation. It also checks to see if the thrusters and sensors are in compliance as defined by `compliance.py`.

It operates by generating macro calls specified by the user-generated YAML configuration files. It currently checks for compliance by ensuring that all sensors and thrusters are in a valid bounding box region and that there are a valid number of each item. See the VRX Technical Guide for specifics on the constraints for the VRX competition, available at the [Documentation Wiki](https://bitbucket.org/osrf/vrx/wiki/documentation).
	
If the thruster/sensor configuration passes, the script creates two xacro files. One is a thruster xacro file in the same directory as the thruster yaml file with the same file name, but with a .xacro extension. The second is a sensor xacro file in the same directory as the sensor yaml file with the same file name, but with a .xacro extension. 

Next, it calls a xacro command to generate the urdf at `wamv_target` using [this file](https://bitbucket.org/osrf/vrx/src/default/wamv_gazebo/urdf/wamv_gazebo.urdf.xacro).

# Important Cases

* When calling `generate_wamv.launch`, if the `thruster_yaml` parameter is not given, then it uses the [default thruster yaml](TODO:put link to file here when merged)

* When calling `generate_wamv.launch`, if the `sensor_yaml` parameter is not given, then it uses the [default sensor yaml](TODO:put link to file here when merged)

* To setup a WAM-V with no thruster and no sensors, you can create empty files `empty_thruster_config.yaml` and `empty_sensor_config.yaml` and then pass them in as parameters to `generate_wamv.launch`

* When `sandisland.launch` or any other simulation launch file is called, if the `urdf` parameter is given, it will use that file for the WAM-V configuration. If the `urdf` parameter is not given, then it will use the default configuration given in the launch file.

## Supported Thrusters and Sensors

Supported thrusters and sensors can be seen in [allowed thrusters](https://bitbucket.org/osrf/vrx/src/default/vrx_gazebo/src/vrx_gazebo_python/generator_scripts/wamv_config/thruster_compliance/numeric.yaml) and [allowed sensors](https://bitbucket.org/osrf/vrx/src/default/vrx_gazebo/src/vrx_gazebo_python/generator_scripts/wamv_config/sensor_compliance/numeric.yaml) (currently as of July 10, 2019 we have only one thruster type).

TODO: Add example of not defining parameters (use default), say that only name is REQUIRED, description about default parameters, where to find them, etc

#Examples
### Example Compliant Yaml Thruster Configuration File 1 
```
engine:
  - prefix: "left"
    position: "-2.373776 1.027135 0.318237"
    orientation: "0.0 0.0 0.0"
  - prefix: "right"
    position: "-2.373776 -1.027135 0.318237"
    orientation: "0.0 0.0 0.0"
```
### Example Compliant Yaml Thruster Configuration File 2 
```
engine:
  - prefix: "left"
    position: "-2.373776 1.027135 0.318237"
    orientation: "0.0 0.0 0.0"
  - prefix: "right"
    position: "-2.373776 -1.027135 0.318237"
    orientation: "0.0 0.0 0.0"

  # Adding new thruster
  - prefix: "middle"
    position: "0 0 0.318237"
    orientation: "0.0 0.0 0.0"
```
### Example Non-compliant Yaml Thruster Configuration File
```
engine:
  - prefix: "left"
    position: "-2.373776 1.027135 0.318237"
    orientation: "0.0 0.0 0.0"
  - prefix: "right"
    position: "-2.373776 -1.027135 0.318237"
    orientation: "0.0 0.0 0.0"

  # Adding new thruster in non-compliant position
  - prefix: "second_right"
    position: "-2.373776 -1.027135 0.318237"
    orientation: "0.0 0.0 0.0"
```

###Example Compliant Yaml Sensor Configuration File 1
```
wamv_camera:
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
lidar:
    - name: lidar_wamv
      type: 16_beam
      P: ${radians(8)}
```

###Example Compliant Yaml Sensor Configuration File 2
```
# Removed all cameras

wamv_gps:
    - name: gps_wamv
      x: -0.85
wamv_imu:
    - name: imu_wamv
      y: -0.2
wamv_p3d:
    - name: p3d_wamv
lidar:
    - name: lidar_wamv
      type: 16_beam
      P: ${radians(8)}
```

###Example Non-compliant Yaml Sensor Configuration File
```
# Too many cameras
wamv_camera:
    - name: front_left_camera
      x: 0.75
      y: 0.1
      P: ${radians(15)}
    - name: front_right_camera
      x: 0.75
      y: -0.1
      P: ${radians(15)}
    - name: front_far_left_camera
      x: 0.75
      y: 0.3
      P: ${radians(15)}
    - name: front_far_right_camera
      x: 0.75
      y: -0.3
      P: ${radians(15)}
    - name: middle_left_camera
      x: 0.6
      y: 0.4
      P: ${radians(15)}
      Y: ${radians(90)}
      post_Y: ${radians(90)}
```