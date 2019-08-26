# Propulsion: Thruster Articulation #

## Description  ##

The VRX simulator has thrusters that allow for thruster articulation, which refers to its ability to rotate and thus redirect the direction of its thrust. As of June 17, 2019, this means that you can set `<enableAngle>true</enableAngle>` in `wamv_gazebo_thruster_config.xacro` to control the angle of the thrusters during simulation. This allows users to publish angle commands (in radians) to the `angleTopic` (defined in `wamv_gazebo_thruster_config.xacro`) to control the thruster angle dynamically, rather than having the thruster stay at a fixed angle with respect to the boat.

## Demonstration ##

1. Launch the example world:


    ```
       $ roslaunch vrx_gazebo sandisland.launch
    ```

2. Look at the WAM-V in sand island. It should have the default thruster configuration and sensor configuration for `sandisland.launch` (as of June 17, 2019, default is H thruster configuration and no sensors). 

    ![Default_Sensors_And_Thrusters.png](https://bitbucket.org/repo/BgXLzgM/images/2154255799-Default_Sensors_And_Thrusters.png)

    You can click _View => Transparent_ to see the thrusters. They should both be pointing straight backwards in the negative x-direction with respect to the boat. 

3. Close gazebo
4. Open the `wamv_gazebo_thruster_config.xacro` file:


    ```
       $ roscd wamv_gazebo/urdf/thruster_layouts
    ```


    ```
       $ gedit wamv_gazebo_thruster_config.xacro
    ```

5. You should see a file similar to the example at the bottom of this page. Copy the example at the bottom of this page into this file (overwrite what is currently there, accurate as of June 17, 2019). The parameters relevant to thruster_articulation are: 

    * `<angleTopic>`: The topic name that the thruster will subscribe to (only subscribes if `<enableAngle>` is true)
    * `<enableAngle>`: If true, the thruster will subscribe to `<angleTopic>` and change its angle accordingly. If false, the thruster will not subcribe to `<angleTopic>` and will remain fixed in its angle with respect to the boat
    * `<maxAngle>`: The absolute value of the maximum thruster angle [radians] If a larger absolute value is published, it will be clipped to be in either the range [-maxAngle, maxAngle] or [-pi, pi] (whichever is smaller)


6. Launch the example world:

    ```
       $ roslaunch vrx_gazebo sandisland.launch
    ```

    You can click _View => Transparent_ to see the thrusters.

7. Observe the ros topics being published and confirm you see the thruster angle topics you want:

    ```
       $ rostopic list
    ```

8. Create an rqt_publisher. This will open a new window similar to the one shown below. Click on the _topic_ dropdown menu and choose the desired thruster angle/command topic, then press the green plus sign on the right. Do this for all desired topics, as shown below. 


    ```
       $ rosrun rqt_publisher rqt_publisher
    ```


    ![RQT_Publisher.png](https://bitbucket.org/repo/BgXLzgM/images/3233543218-RQT_Publisher.png)


9. Test publishing `2` for the thrust_cmd and `0` for the thrust_angle by clicking on the drop-down arrow next to the topic, double clicking the expression value for the topic, and changing the number. Be sure to click the box to fill it with a checkmark to start publishing. The WAM-V should move forward normally, as it would with fixed thrusters. Note: if there is wind in the simulation, you might not see the WAM-V. You can click _CTRL+SHIFT+R_ to reset the WAM-V to the start position.

    ![RQT_PUB_1.png](https://bitbucket.org/repo/BgXLzgM/images/2140541476-RQT_PUB_1.png)

10. Test publishing `2` for the thrust_cmd and `1.57` for the thrust_angle. This should rotate the thruster by 90 degrees (pi/2 radians) in the counter-clockwise (positive) direction, as shown below. The force from the thrust should match what is shown visually, making it spin in the clockwise direction.

    ![90 Degree Thrusters.png](https://bitbucket.org/repo/BgXLzgM/images/1573770290-90%20Degree%20Thrusters.png)

11. Test publishing `2` for the thrust_cmd and `-10` for the thrust_angle. We have set `<maxAngle>${pi/2}</maxAngle>`, so `-10` will be clipped to `${-pi/2}` or `-1.57`. This should rotate the thruster by -90 degrees (-pi/2 radians) in the clockwise (negative) direction, as shown below. The force from the thrust should match what is shown visually, making it spin in the counter-clockwise direction.

    ![Neg 90 Degree Thrusters.png](https://bitbucket.org/repo/BgXLzgM/images/1788754463-Neg%2090%20Degree%20Thrusters.png)

12. Close gazebo

Note:

* The thruster angle is controlled by a pre-tuned PID controller at the rotating joint. It does not move the angle immediately to the position being subscribed to, but uses a PID controller to send force commands to the joint to reach the desired angle quickly and accurately. The PID constants can be tuned for different desired use.

* As of June 30, 2019, the thruster angles can be controlled by `usv_keydrive.launch` or `usv_joydrive.launch`. Read [Driving](https://bitbucket.org/osrf/vrx/wiki/tutorials/Driving) for more information.

# Example WAM-V Gazebo Thruster Configuration File
```
<?xml version="1.0"?>
<plugin xmlns:xacro="http://ros.org/wiki/xacro">
  <xacro:macro  name="wamv_gazebo_thruster_config" params="name">
    <thruster>
      <!-- Required Parameters -->
      <linkName>${name}_propeller_link</linkName>
      <propJointName>${name}_engine_propeller_joint</propJointName>
      <engineJointName>${name}_chasis_engine_joint</engineJointName>
      <cmdTopic>${name}_thrust_cmd</cmdTopic>
      <angleTopic>${name}_thrust_angle</angleTopic>
      <enableAngle>true</enableAngle>

      <!-- Optional Parameters -->
      <mappingType>1</mappingType>
      <maxCmd>1.0</maxCmd>
      <maxForceFwd>250.0</maxForceFwd>
      <maxForceRev>-100.0</maxForceRev>
      <maxAngle>${pi/2}</maxAngle>
    </thruster>
  </xacro:macro>
</plugin>
```