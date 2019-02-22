## Run the VRX image from Docker Hub ##

* Create a VRX docker directory (if you haven't already) and download the run script:

```
#!bash
    $ mkdir -p ~/vrx/docker && cd ~/vrx/docker
    $ wget https://bitbucket.org/osrf/vrx/raw/default/docker/run.bash
    $ chmod u+x run.bash
```

* Run the VRX container provided on [Docker Hub](https://hub.docker.com/r/osrf/vrx/tags/).  This step will attempt to run the current image.  If the image isn't already on your system (it is the first time it is run, or there is an update) docker will download the image.  The images are large (~1 GB), so beware this may take some time.

    * Option1: To use the default Docker runtime (no Nvidia):

            $ ./run.bash osrf/vrx:current

    * Option2: For the Nvidia runtime, use the -n flag:

            $ ./run.bash -n osrf/vrx:nvidia_current

* Note that if this is the first time running the image and you did not build it on your machine, the script will first pull the image from the repository to your machine, which may take a moment.

* If all has gone well, the command will drop you into a bash prompt running as the user "developer" inside the container.

## Test your VRX installation: ##

```
#!bash
    $ roslaunch vrx_gazebo sandisland.launch
```

* If everything is working properly, Gazebo should launch Sand Island world with a simple environment.

* If you encounter errors, try the [Troubleshooting](https://bitbucket.org/osrf/vrx/wiki/Troubleshooting) Setup and Install page.