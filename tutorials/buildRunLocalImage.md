## Build the VRX image using latest repository code ##

* Create a build directory for the image:

        $ mkdir -p ~/vrx/docker/vrx && cd ~/vrx/docker/vrx

* Download the appropriate vrx Dockerfile, depending on whether you are using the Nvidia runtime.

    * Option 1: For the Nvidia enabled version:

            $ wget https://bitbucket.org/osrf/vrx/raw/default/docker/vrx_nvidia/Dockerfile

    * Option 2: Otherwise, if you are running without an Nvidia GPU, use:

            $ wget https://bitbucket.org/osrf/vrx/raw/default/docker/vrx/Dockerfile

* Download the build and run scripts to the parent of your build directory:

```
#!bash
    $ cd ..
    $ wget https://bitbucket.org/osrf/vrx/raw/default/docker/build.bash
    $ wget https://bitbucket.org/osrf/vrx/raw/default/docker/run.bash
    $ chmod u+x build.bash run.bash
```

* Build your vrx Docker image.  This step downloads, installs and compiles number of packages, so it will take some time, e.g., 10's of minutes to an hour depending on internet connection speed.

        $ ./build.bash vrx


## Run the vrx image ##

* Option 1: To use the default Docker runtime (no Nvidia):

        $ ./run.bash vrx

* Option 2: For the nvidia runtime, use the -n flag:

        $ ./run.bash -n vrx

* If all has gone well, the command will drop you into a bash prompt running as the user "developer" inside the container.

## Test your vrx installation: ##

```
#!bash
    $ roslaunch vrx_gazebo sandisland.launch
```

* If everything is working properly, Gazebo should launch Sand Island world with a simple environment.

* If you encounter errors, try the [Troubleshooting](https://bitbucket.org/osrf/vrx/wiki/Troubleshooting) Setup and Install page.