## Build the VRX image using latest repository code ##

Run the appropriate set of commands below, depending on whether Nvidia support is required. Note that for both versions, executing the build script in the second command will download, install and compile a number of packages, so it will take some time, e.g., 10's of minutes to an hour depending on internet connection speed.

### 1. Building Without Nvidia Support ###

For users running without an Nvidia GPU, the commands to build the VRX image are:

        $ hg clone https://bitbucket.org/osrf/vrx
        $ docker/build.bash .
        $ docker/run.bash vrx


### 2. Building With Nvidia Support ###

To build the Nvidia-enabled version, run:

        $ hg clone https://bitbucket.org/osrf/vrx
        $ docker/build.bash -n .
        $ docker/run.bash -n vrx_nvidia


If all has gone well, the final command will drop you into a bash prompt running as the user "developer" inside the container.

## Test your vrx installation: ##

```
#!bash
    $ roslaunch vrx_gazebo sandisland.launch
```

* If everything is working properly, Gazebo should launch Sand Island world with a simple environment.

* If you encounter errors, try the [Troubleshooting](https://bitbucket.org/osrf/vrx/wiki/Troubleshooting) Setup and Install page.

## Next Steps ##
If you plan to use the container you built as your primary environment for running or developing code for VRX, you may want to work through the [Working With Your Docker Image](https://bitbucket.org/osrf/vrx/wiki/tutorials/workingWithDocker) tutorial to make your vrx repository accessible within the container.