## Build the VRX image using latest repository code ##

* Clone the repository and change to the docker directory:
        ```
        #!bash

        $ hg clone https://bitbucket.org/osrf/vrx
        $ cd vrx/docker

## Build Docker image ##

This step downloads, installs and compiles number of packages, so it will take some time, e.g., 10's of minutes to an hour depending on internet connection speed.

* Option 1: For the Nvidia enabled version, use the `-n` flag:

        $ ./build.bash -n .

* Option 2: Otherwise, if you are running without an Nvidia GPU, use:

        $ ./build.bash .

## Run the vrx image ##

* Option 1: For the Nvidia enabled version, use the `-n` flag:

        $ ./run.bash -n vrx_nvidia

* Option 2: Otherwise, if you are running without an Nvidia GPU, use:

        $ ./run.bash vrx

* If all has gone well, the command will drop you into a bash prompt running as the user "developer" inside the container.

## Test your vrx installation: ##

```
#!bash
    $ roslaunch vrx_gazebo sandisland.launch
```

* If everything is working properly, Gazebo should launch Sand Island world with a simple environment.

* If you encounter errors, try the [Troubleshooting](https://bitbucket.org/osrf/vrx/wiki/Troubleshooting) Setup and Install page.