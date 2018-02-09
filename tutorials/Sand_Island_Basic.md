Simple base world and vehicle model with environment plugins.

**Beware - This wiki and the relevant source is under development!  Things may change/break at anytime.
**
# Overview

This tutorial is the "hello world" of running the Gazebo WAM-V model with the associated plugins.  The model is the base setup, the Sand Island world with a simple environment (wind+waves) and a vehicle-only model and thrust plugin.  

A description of the model implementation (hydrodynamics, waves, wind and propulsion simulation) is available at https://github.com/bsb808/usv_gazebo_plugins/blob/waves/docs/theory_of_operation.pdf

Follow-on tutorials describe how to customize this base for particular vehicle and sensor configurations.

# Setup

Please, check the [setup instructions](https://bitbucket.org/osrf/vmrc/wiki/tutorials/SystemSetup) before running this tutorial.

# Run instructions

* Source the ROS setup.bash file:

```
#!bash
    
    $ source /opt/ros/kinetic/setup.bash

```

* Source the `setup.bash` of our VMRC workspace:

```
#!bash
    
    $ source ./devel/setup.bash

```

* Use the provide launch file to run the example:

```
#!bash

    $ roslaunch robotx_gazebo sandisland.launch 
```

You should see a new Gazebo window similar to this one:

![Screenshot from 2018-01-25 10-59-15_mr.png](https://bitbucket.org/repo/BgXLzgM/images/2097879520-Screenshot%20from%202018-01-25%2010-59-15_mr.png)

You will notice that the USV starts drifting along the x-axis.  That is due to the wind.  More on that later.

If you zoom in your will see the WAM-V model like this:

![2349056053-Screenshot from 2018-01-25 11-00-30.png.70.png](https://bitbucket.org/repo/BgXLzgM/images/2394960890-2349056053-Screenshot%20from%202018-01-25%2011-00-30.png.70.png)

This visual and collision model is under development.  We are hoping to both improve the visual representation and make it more computationally efficient.

The shoreline is a coarse approximation of the RobotX competition area, Sand Island, Honolulu, HI.  

![map_colage.png](https://bitbucket.org/repo/BgXLzgM/images/869375701-map_colage.png)

There navigation course task is represented by the two pairs of red and green buoys.

![3195009535-out.png](https://bitbucket.org/repo/BgXLzgM/images/3465846643-3195009535-out.png)