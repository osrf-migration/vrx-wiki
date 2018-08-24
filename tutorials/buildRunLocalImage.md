## Build the VMRC image ##

* Create a build directory for the image:

        $ mkdir -p ~/vmrc_docker/vmrc && cd ~/vmrc_docker/vmrc

* Download the appropriate vmrc Dockerfile, depending on whether you are using the Nvidia runtime. 

    * For the Nvidia enabled version:

            $ wget https://bitbucket.org/osrf/vmrc/raw/default/docker/vmrc_nvidia/Dockerfile

    * Otherwise, if you are running without an Nvidia GPU, use:

            $ wget https://bitbucket.org/osrf/vmrc/raw/default/docker/vmrc/Dockerfile

* Download the build and run scripts to the parent of your build directory:

```
#!bash
    $ cd ..
    $ wget https://bitbucket.org/osrf/vmrc/raw/default/docker/build.bash
    $ wget https://bitbucket.org/osrf/vmrc/raw/default/docker/run.bash
    $ chmod u+x build.bash run.bash
```

* Build your VMRC Docker image:

        $ ./build.bash vmrc


## Run the VMRC image ##

* To use the default Docker runtime (no Nvidia):

        $ ./run.bash vmrc

* For the nvidia runtime, use the -n flag:

            $ ./run.bash -n vmrc

* If all has gone well, the command will drop you into a bash prompt running as the user "developer" inside the container. 

## Test your VMRC installation: ##

```
#!bash
    $ roslaunch robotx_gazebo sandisland.launch
```

* If everything is working properly, Gazebo should launch.

* If you encounter errors, try the [Troubleshooting](https://bitbucket.org/osrf/vmrc/wiki/Troubleshooting) Setup and Install page.