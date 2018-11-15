# Overview

This tutorial is the "hello world" of running the Gazebo WAM-V model with the associated plugins.  The model is the base setup: the Sand Island world with a simple environment (wind+waves) and a vehicle-only model and thrust plugin.  

A description of the model implementation (hydrodynamics, waves, wind and propulsion simulation) is available at https://github.com/bsb808/robotx_docs/blob/master/theoryofoperation/theory_of_operation.pdf

Follow-on tutorials describe how to customize this base for particular vehicle and sensor configurations.

# Setup

Please make sure you've set up your system using the instructions for [Host-Based Installation](https://bitbucket.org/osrf/vmrc/wiki/tutorials/SystemSetupInstall) or [Container-Based Installation](https://bitbucket.org/osrf/vmrc/wiki/tutorials/SystemSetupDocker) before running this tutorial.

# Run instructions

* Source the ROS setup.bash file:

```
#!bash
    
    $ source /opt/ros/kinetic/setup.bash

```

* Source the `setup.bash` of our VMRC workspace:

```
#!bash
    
    $ cd ~/vmrc_ws
    $ source ./devel/setup.bash

```

* Use the provide launch file to run the example:

```
#!bash

    $ roslaunch vmrc_gazebo sandisland.launch light_buoy:=false

```

You should see a new Gazebo window similar to this one:

![Screenshot from 2018-01-25 10-59-15_mr.png](https://bitbucket.org/repo/BgXLzgM/images/2097879520-Screenshot%20from%202018-01-25%2010-59-15_mr.png)

You may notice that the USV starts drifting along the x-axis.  That is due to the wind.  More on that later.

If you zoom in you will see the WAM-V model like this:

![Screenshot from 2018-02-26 08-55-16.png](https://bitbucket.org/repo/BgXLzgM/images/4154749175-Screenshot%20from%202018-02-26%2008-55-16.png)

The shoreline is a coarse approximation of the RobotX competition area, Sand Island, Honolulu, HI.  

![map_colage.png](https://bitbucket.org/repo/BgXLzgM/images/869375701-map_colage.png)

There navigation course task is represented by the two pairs of red and green buoys.

![3195009535-out.png](https://bitbucket.org/repo/BgXLzgM/images/3465846643-3195009535-out.png)