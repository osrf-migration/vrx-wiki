# Overview

This tutorial is the "hello world" of running the Gazebo WAM-V model with the associated plugins.  The model is the base setup: the Sand Island world with a simple environment (wind+waves) and a vehicle-only model and thrust plugin.

A description of the model implementation (hydrodynamics, waves, wind and propulsion simulation) is available at https://github.com/bsb808/robotx_docs/blob/master/theoryofoperation/theory_of_operation.pdf

Follow-on tutorials describe how to customize this base for particular vehicle and sensor configurations.

# Setup

Please make sure you've set up your system using the instructions for [Host-Based Installation](https://bitbucket.org/osrf/vrx/wiki/tutorials/SystemSetupInstall) or [Container-Based Installation](https://bitbucket.org/osrf/vrx/wiki/tutorials/SystemSetupDocker) before running this tutorial.

# Run instructions

* Source the ROS setup.bash file:

```
#!bash

    $ source /opt/ros/melodic/setup.bash

```

* Only needed if you built from source:

```
#!bash

    $ cd ~/vrx_ws
    $ source ./devel/setup.bash

```

* Use the provided launch file to run the example:

```
#!bash

    $ roslaunch vrx_gazebo sandisland.launch

```

You should see a new Gazebo window similar to this one:

![Screenshot from 2019-02-28 09-52-58_mr.png](https://bitbucket.org/repo/BgXLzgM/images/250474638-Screenshot%20from%202019-02-28%2009-52-58_mr.png)

You may notice that the USV starts drifting along the x-axis.  That is due to the wind.  More on that later.

If you zoom in you will see the WAM-V model like this:

![Screenshot from 2018-02-26 08-55-16.png](https://bitbucket.org/repo/BgXLzgM/images/4154749175-Screenshot%20from%202018-02-26%2008-55-16.png)

The shoreline is a coarse approximation of the RobotX competition area, Sand Island, Honolulu, HI.

![map_colage.png](https://bitbucket.org/repo/BgXLzgM/images/869375701-map_colage.png)

The navigation course task is represented by the two pairs of red and green buoys.

![3195009535-out.png](https://bitbucket.org/repo/BgXLzgM/images/3465846643-3195009535-out.png)