# Tutorials for the Virtual RobotX Challenge (VRX)

We recommend following the tutorials in numerical order. At the end of these tutorials you will be familiar with Gazebo, ROS, and how to use VRX.

1. System Setup: Guides for configuring your personal PC and installing software for VRX.  Make sure to review the [System Requirements](https://bitbucket.org/osrf/vrx/wiki/system_requirements) before you begin.

    * [Install on Host](https://bitbucket.org/osrf/vrx/wiki/tutorials/SystemSetupInstall): Install ROS, Gazebo and the current VRX repositories on your Ubuntu host.  Use this option if your computer satisfies all of the hardware requirements in [System Requirements](https://bitbucket.org/osrf/vrx/wiki/system_requirements) and you are willing to setup your system with the specific version of Ubuntu/ROS/Gazebo for VRX.

    * [Run Using Docker](https://bitbucket.org/osrf/vrx/wiki/tutorials/SystemSetupDocker): Install Docker on your Linux host and run ROS, Gazebo and the latest VRX code within a Docker container.  Use this option if you are using the same host for many development projects that make use of different software environments, or just prefer to leave your host system untouched.

1. Examples:

    * [Sand Island Basic](https://bitbucket.org/osrf/vrx/wiki/tutorials/Sand_Island_Basic): A simple base world with a WAM-V model.
    * [VRX](https://bitbucket.org/osrf/vrx/wiki/tutorials/ExampleVrx): Example of a specific WAM-V and sensor configuration similar to what teams have used for RobotX. Also illustrates using Rviz.

1. [Adding Course Elements](https://bitbucket.org/osrf/vrx/wiki/tutorials/Adding%20course%20elements): Creating your own course in gazebo.

1. [Driving](https://bitbucket.org/osrf/vrx/wiki/tutorials/Driving): Methods for making the WAM-V move using teleoperation (keyboard or gamepad) or programmatically.

1. [Propulsion Configuration](https://bitbucket.org/osrf/vrx/wiki/tutorials/PropulsionConfiguration): Examples of existing thruster configurations and methods for creating custom configurations.

1. [Adding Sensors to the WAM-V Base](https://bitbucket.org/osrf/vrx/wiki/tutorials/AddingSensors): Example of customizing the base USV by adding sensors (GPS, camera, etc.).

1. [Visualizing with RVIZ](https://bitbucket.org/osrf/vrx/wiki/tutorials/Visualizing%20with%20RVIZ): See the WAM-V and sensors in rviz.

1. [Changing Simulation Parameters](https://bitbucket.org/osrf/vrx/wiki/tutorials/ChangingPluginParameters): How to change model parameters (hydrodynamics, thrust, etc.) or environmental parameters (wind, waves, etc.) to customize your scenario.

1. [How to create a custom dock](https://bitbucket.org/osrf/vrx/wiki/tutorials/CreateDocks): Create a new dock with your own shape and dimensions.

1. [Troubleshooting](https://bitbucket.org/osrf/vrx/wiki/Troubleshooting): Diagnosing and fixing common problems with VRX.