## Build the VMRC image using latest repository code ##

* Create a build directory for the image:

        $ mkdir -p ~/vmrc/docker/vmrc && cd ~/vmrc/docker/vmrc

* Download the appropriate vmrc Dockerfile, depending on whether you are using the Nvidia runtime. 

    * Option 1: For the Nvidia enabled version:

            $ wget https://bitbucket.org/osrf/vmrc/raw/default/docker/vmrc_nvidia/Dockerfile

    * Option 2: Otherwise, if you are running without an Nvidia GPU, use:

            $ wget https://bitbucket.org/osrf/vmrc/raw/default/docker/vmrc/Dockerfile

* Download the build and run scripts to the parent of your build directory:

```
#!bash
    $ cd ..
    $ wget https://bitbucket.org/osrf/vmrc/raw/default/docker/build.bash
    $ wget https://bitbucket.org/osrf/vmrc/raw/default/docker/run.bash
    $ chmod u+x build.bash run.bash
```

* Build your VMRC Docker image.  This step downloads, installs and compiles number of packages, so it will take some time, e.g., 10's of minutes to an hour depending on internet connection speed.

        $ ./build.bash vmrc


## Run the VMRC image ##

* Option 1: To use the default Docker runtime (no Nvidia):

        $ ./run.bash vmrc

* Option 2: For the nvidia runtime, use the -n flag:

        $ ./run.bash -n vmrc

* If all has gone well, the command will drop you into a bash prompt running as the user "developer" inside the container. 

## Test your VMRC installation: ##

```
#!bash
    $ roslaunch vmrc_gazebo sandisland.launch
```

* If everything is working properly, Gazebo should launch Sand Island world with a simple environment.

* If you encounter errors, try the [Troubleshooting](https://bitbucket.org/osrf/vmrc/wiki/Troubleshooting) Setup and Install page.