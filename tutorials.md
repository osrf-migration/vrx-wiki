# Tutorials for the Virtual Maritime RobotX Challenge (VMRC)

We recommend following the tutorials in numerical order. At the end of these tutorials you will be familiar with Gazebo, ROS, and how to use VMRC.

1. [System Setup](https://bitbucket.org/osrf/vmrc/wiki/tutorials/SystemSetup): Guides for configuring your personal PC and installing software for VMRC.  Make sure to review the [System Requirements](https://bitbucket.org/osrf/vmrc/wiki/system_requirements) before you begin.

    * [Install on Host](https://bitbucket.org/osrf/vmrc/wiki/tutorials/SystemSetupInstall): Install ROS, Gazebo and the current VMRC repositories on your Ubuntu host.  Use this option if your computer satisfies all of the hardware requirements in [System Requirements](https://bitbucket.org/osrf/vmrc/wiki/system_requirements) and you are willing to setup your system with the specific version of Ubuntu/ROS/Gazebo for VMRC. 
    * [Create Docker Container](https://bitbucket.org/osrf/vmrc/wiki/tutorials/SystemSetupDocker): Create a Docker container on your Linux host and install ROS, Gazebo and latest VMRC repositories within the container.  Use this option if you are using the same host for many development projects that make use of different software environments.
    * Use Pre-Configured Docker Container: Coming Soon...  Use this option if you would like the quickest method for getting the standard VMRC environment up and running.

2. [Sand Island Basic](https://bitbucket.org/osrf/vmrc/wiki/tutorials/Sand_Island_Basic): A simple base world with a WAM-V model.

3. [Driving](https://bitbucket.org/osrf/vmrc/wiki/tutorials/Driving): Methods for making the WAM-V move using teleoperation (keyboard or gamepad) or programmatically.

4. [Changing Simulation Parameters](https://bitbucket.org/osrf/vmrc/wiki/tutorials/ChangingPluginParameters): How to change model parameters (hydodynamics, thrust, etc.) or environmental parameters (wind, waves, etc.) to customize your scenario.

5. [Adding Sensors to the WAM-V Base](https://bitbucket.org/osrf/vmrc/wiki/tutorials/AddingSensors): Example of customizing the base USV by adding sensors (GPS, camera, etc.)

6. [Troubleshooting](https://bitbucket.org/osrf/vmrc/wiki/Troubleshooting)