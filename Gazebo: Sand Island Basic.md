Simple base world and vehicle model with environment plugins.

**Beware - This wiki and the relevant source is under development!  Things may change/break at anytime.
**

# Setup

Please, check the [setup instructions](https://bitbucket.org/osrf/vmrc/wiki/tutorials/SystemSetup) before running this tutorial.

# Run instructions

To run the example, use the provide launch file...

```
roslaunch robotx_gazebo sandisland.launch 
```

You should see a new Gazebo window something like this...
![Screenshot from 2018-01-25 10-59-15_mr.png](https://bitbucket.org/repo/BgXLzgM/images/2097879520-Screenshot%20from%202018-01-25%2010-59-15_mr.png)
You will notice that the USV starts drifting along the x-axis.  That is due to the wind.  More on that later.

If you zoom in your will see the WAM-V model like this...
![Screenshot from 2018-01-25 11-00-30.png.70.png](https://bitbucket.org/repo/BgXLzgM/images/2349056053-Screenshot%20from%202018-01-25%2011-00-30.png.70.png)
This visual and collision model is under development.  We are hoping to both improve the visual representation and make it more computationally efficient.

The shoreline is a coarse approximation of the RobotX competition area, Sand Island, Honolulu, HI.  
![map_colage.png](https://bitbucket.org/repo/BgXLzgM/images/869375701-map_colage.png)

There navigation course task is represented by the two pairs of red and green buoys.
![out.png](https://bitbucket.org/repo/BgXLzgM/images/3195009535-out.png)

# Status

Currently the following is implemented

 * Buoy buoyancy using custom fork of standard Gazebo buoyancy plugin.
 * No sensors
 * USV Dynamics via usv_gazebo_plugins.  Parameters defined in usv_gazebo_plugins/urdf/wamv_gazebo_dynamics_plugin.xacro
   * Hydrodynamic parameters based on publications from FAU and engineering judgement
   * Wind implementation via same plugin
   * Thrust is linear with thrust command
  * Visual and collision meshes in wamv_description.  Better ones coming soon
  * Buoy meshes in robotx_gazebo.

## Coming Soon

 * Improved meshes for WAM-V, buoys and other RobotX objects
 * Simulation of simple wave field forcing
 * Basic acoustics model
 * Tutorials on how to add sensors
 * Improved hydrodynamic parameter estimates of WAM-V
 * User defined thrust profile for propulsion
 * Docker container