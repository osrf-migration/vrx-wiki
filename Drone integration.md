# Installing vcstool and colcon #

For getting the sources of all libraries the easiest way is to use vcstool. The tool is available from pip in all platforms:


```
#!c++

pip install vcstool
```

To compile all the different libraries and ign-gazebo in the right order it is recommended to use colcon. The colcon tool is available in all platforms using pip:


```
#!c++

pip install -U colcon-common-extensions
```


# Getting the sources #

The instructions bellow use some UNIX commands to manage directories but the obvious alternatives on Windows should provide the same result.

The first step would be to create a developer workspace in which vcstool and colcon can work.


```
#!c++

mkdir -p /data/ignition_ws/src
cd /data/ignition_ws/src
```


All the sources of ignition dependencies are declared in a subt.yaml file. Create it with the following content:


```
#!c++

repositories:
   ign-cmake:
      type: hg
      url: https://bitbucket.org/ignitionrobotics/ign-cmake
      version: ign-cmake2
   ign-common:
      type: hg
      url: https://bitbucket.org/ignitionrobotics/ign-common
      version: ign-common3
   ign-fuel-tools:
      type: hg
      url: https://bitbucket.org/ignitionrobotics/ign-fuel-tools
      version: ign-fuel-tools3
   ign-gazebo:
      type: hg
      url: https://bitbucket.org/ignitionrobotics/ign-gazebo
      version: default
   ign-gui:
      type: hg
      url: https://bitbucket.org/ignitionrobotics/ign-gui
      version: default
   ign-launch:
      type: hg
      url: https://bitbucket.org/ignitionrobotics/ign-launch
      version: fixing_version_info
   ign-math:
      type: hg
      url: https://bitbucket.org/ignitionrobotics/ign-math
      version: ign-math6
   ign-msgs:
      type: hg
      url: https://bitbucket.org/ignitionrobotics/ign-msgs
      version: default
   ign-physics:
      type: hg
      url: https://bitbucket.org/ignitionrobotics/ign-physics
      version: ign-physics1
   ign-plugin:
      type: hg
      url: https://bitbucket.org/ignitionrobotics/ign-plugin
      version: ign-plugin1
   ign-rendering:
      type: hg
      url: https://bitbucket.org/ignitionrobotics/ign-rendering
      version: default
   ign-sensors:
      type: hg
      url: https://bitbucket.org/ignitionrobotics/ign-sensors
      version: default
   ign-tools:
      type: hg
      url: https://bitbucket.org/ignitionrobotics/ign-tools
      version: default
   ign-transport:
      type: hg
      url: https://bitbucket.org/ignitionrobotics/ign-transport
      version: default
   sdformat:
      type: hg
      url: https://bitbucket.org/osrf/sdformat
      version: sdf8
   subt:
      type: hg
      url: https://bitbucket.org/osrf/subt
      version: ign_comms

```

And then run:


```
#!c++

vcs import < subt.yaml
```


The *src* subdirectory should contain all the sources ready to be built.

# Building the Ignition Libraries #

Once all the sources are in place it is time to compile them. Start the procedure by changing into the workspace and listing the packages recognized by colcon:


```
#!c++

cd /data/ignition_ws
```

Now you are ready to build the whole set of libraries:


```
#!c++

colcon build
```

# Using the workspace #

The workspace binaries are ready but every time that ign-gazebo needs to be executed or third party code is going to be developed using the Ignition libraries, one command is needed:


```
#!c++

. install/local_setup.bash
```

After running the command all paths for running apps or developing code will be set in the current shell.

# Other tweaks required #


```
#!c++

export IGN_GAZEBO_RESOURCE_PATH=/data/ignition_ws/install/subt_ign/share/subt_ign/worlds
export IGN_CONFIG_PATH=/data/ignition_ws/install/ignition-launch1/share/ignition
export IGN_LAUNCH_PLUGIN_PATH=/data/ignition_ws/install/subt_ign/lib
export LD_LIBRARY_PATH=/data/ignition_ws/install/subt_ign/lib:/data/ignition_ws/install/subt_communication_broker_ign/lib:$LD_LIBRARY_PATH
```

# Launch SubT with Ignition Gazebo


```
#!c++

ign launch -f install/subt_ign/share/subt_ign/launch/competition.ign -v 4 &
ign launch -f install/subt_ign/share/subt_ign/launch/team.ign -v 4
```


# Stop SubT


```
#!c++

fg
<CTRL-C>
```