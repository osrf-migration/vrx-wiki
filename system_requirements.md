# System Requirements for Running VRX #

## Recommended system setup

### Hardware ###

In order to run VRX we recommend your computing hardware has:

- Modern multi-core CPU, e.g. Intel Core i5
- 8 Gb of RAM
- Discrete Graphics Card, e.g. Nvidia GTX 650

The system will run without the GPU, but the Gazebo simulation will run much faster (should run in real-time) with access to a GPU.  Without a GPU the simulation is likely to run slower than real-time.

### Software ###

The [system setup tutorials](https://bitbucket.org/osrf/vrx/wiki/tutorials) provide instructions for installing this environment directly on your host computer or by using a Docker container.

The VRX software support is provided for the following software environment:

- Ubuntu Desktop 18.04 Bionic (64-bit)
- Gazebo 9.9.0+ (<9.9.0 is not sufficient)
- ROS Melodic

### Peripherals ###

We also recommend having a gamepad for testing when driving the WAM-V in the simulated world. In the examples we use a Logitech F310 ([walmart](https://www.walmart.com/ip/Logitech-F310-GamePad/16419686)).