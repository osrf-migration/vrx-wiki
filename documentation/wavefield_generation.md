Thanks to the contribution from @Rhys Mainwaring (see [PR #78](https://bitbucket.org/osrf/vrx/pull-requests/78/issue-23-coordinate-the-physics-and/diff) and [Issue #23](https://bitbucket.org/osrf/vrx/issues/23/coordinate-the-physics-and-visualization) ) the VRX environment implements a model of ocean surface waves that synchronizes the visual representation with the physical influence on models within the environment.  This page is intended to explain the implementation.

# Models #

## Ocean Model #

The `wave_gazebo/world_models/ocean_waves` model includes three plugins:

  1. **WavefieldModelPlugin:ModelPlugin** 
    * Includes instances of 
        * WavefieldEntity:gazebo::physics::Base which includes an instance of
            * WaveParameters (defined in Wavefield.hh) holds the current values to define the wave field (number, angle, scale, etc.)
    * Subscribed Gazebo Topics:
        * ~/request ([gazebo::msgs::Request](https://bitbucket.org/osrf/gazebo/src/default/gazebo/msgs/request.proto))  Responds with a Param_V of all of the wave parameters (number, angle, scale, etc.)
        * ~/wave ([gazebo::msgs::Param_V](https://bitbucket.org/osrf/gazebo/src/default/gazebo/msgs/param_v.proto))  - Allows for setting the values of the WaveParameters instance via gazebo topic.  The WaveMsgPublisher utility is supplied to support.
    * Published Gazebo Topics
        * ~/response ([gazebo::msgs::Response](https://bitbucket.org/osrf/gazebo/src/default/gazebo/msgs/response.proto))
  1. **WavefieldVisualPlugin:VisualPlugin**  
     * On initialization, requests wave parameters from the WavefieldModelPlugin via Gazebo ~/request message.
     * During the visual plugin update, uses rendering API to set wave parameters to OpenGL shader GernstnerWaves.vert. 
         * Done using the Visual::SetMaterialShaderParam to pass the simulation time value to the GernstnerWaves.vert vertex shader program where the wave model is run to generate the 3D wave field shape.  This feature doesn't seem to be terribly well documented, but here is the [PR](https://bitbucket.org/osrf/gazebo/pull-requests/2863/add-visual-setmaterialshaderparam-function/diff) that implemented the feature and an example.
         * Note that it appears that the GernstnerWaves.vert is hardcoded to 3 component waves.
     * Subscribed Gazebo Topics:
         * ~/response ([gazebo::msgs::Response](https://bitbucket.org/osrf/gazebo/src/default/gazebo/msgs/response.proto)). When receives a response from the model plugin, sets parameters to the vertex shader.
         * ~/wave ([gazebo::msgs::Param_V](https://bitbucket.org/osrf/gazebo/src/default/gazebo/msgs/param_v.proto))  - Allows for setting the values of the WaveParameters instance via gazebo topic.  The WaveMsgPublisher utility is supplied to support.
         * ~/world_stats 
    * Published Gazebo Topics
         * ~/request ([gazebo::msgs::Request](https://bitbucket.org/osrf/gazebo/src/default/gazebo/msgs/request.proto)) Requests wave_param wave parameters.
  1. A second **WavefieldVisualPlugin:VisualPlugin** for below the water surface. Uses the same parameters and shader.

## WAM-V USV Model ##

  1. **UsvDynamicsPlugin:ModelPlugin** 
    * Uses the `wave_model` parameter to specify a model, by name, that includes an instance of the WavefieldModelPlugin
    * On Update
        * Uses the WavefieldModelPlugin API to get WaveParameters pointer so that this plugin is using the same parameters as used by the visual plugin.
            * This is done on each update, which seems like overkill.  If we consider the wave parameters to be constant for a simulation run this could be simplified.
        * Calls a the static WavefieldSampler::ComputDepthDirectly function (see Wavefield.hh/cc) to implement the geometry of the wave height model at specific grid points.

## Buoys and Obstacle Models (E.g, surmark950400) ##


  1. **UsvDynamicsPlugin:ModelPlugin** 
    * Same implementation for wave height as done for the UsvDynamicsPlugin
        * Uses the `wave_model` parameter to specify a model, by name, that includes an instance of the WavefieldModelPlugin. Then on update retrieves the current WaveParameters pointer via the WavefieldModelPlugin and determines the water level at simulation time for the point location of the link via the WavefieldSampler::ComputDepthDirectly function.