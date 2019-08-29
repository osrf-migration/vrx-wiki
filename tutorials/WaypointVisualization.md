# Waypoint visualization markers

Waypoint visualization allows users to see waypoint markers for station-keeping and wayfinding tasks inside Gazebo. The visualization markers only show the position of the waypoints and not their orientation.

![2598206705-wayfinding.jpg](https://bitbucket.org/repo/BgXLzgM/images/3707657839-2598206705-wayfinding.jpg)

Waypoint visualization is enabled by default. Parameters for markers can be modified by editing the SDF in `stationkeeping_task.world.xacro` or `wayfinding_task.world.xacro` inside `vrx_gazebo/worlds`.

```
<markers>
   <material>Gazebo/Green</material>
   <scaling>0.2 0.2 2.0</scaling>
   <height>0.5</height>
</markers>
```
To disable the visualization, simply remove the `<markers />` tag.

### SDF Parameters
* `<material />` - specifies the Gazebo material used for the marker. (Default: Gazebo/red)
* `<scaling />` - specifies scaling for the marker. (Default: 0.2 0.2 1.5)
* `<height />` - height of marker above water (Default: 0)

*Note: all parameters are optional*