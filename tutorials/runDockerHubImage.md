## Run the VMRC image ##

* Create a vmrc docker directory (if you haven't already) and download the run script:

```
#!bash
    $ mkdir -p ~/vmrc_docker && cd ~/vmrc_docker
    $ wget https://bitbucket.org/osrf/vmrc/raw/default/docker/run.bash
    $ chmod u+x run.bash
```

* Run the VMRC container provided on Docker Hub:

    * To use the default Docker runtime (no Nvidia):

            $ ./run.bash osrf/vmrc:current

    * For the nvidia runtime, use the -n flag:

            $ ./run.bash -n osrf/vmrc:nvidia_current

* Note that if this is the first time running the image and you did not build it on your machine, the script will first pull the image from the repository to your machine, which may take a moment.

* If all has gone well, the command will drop you into a bash prompt running as the user "developer" inside the container. 

## Test your VMRC installation: ##

```
#!bash
    $ roslaunch robotx_gazebo sandisland.launch
```

* If everything is working properly, Gazebo should launch.

* If you encounter errors, try the [Troubleshooting](https://bitbucket.org/osrf/vmrc/wiki/Troubleshooting) Setup and Install page.