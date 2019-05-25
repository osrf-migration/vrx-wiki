Thanks to the contribution from @Rhys Mainwaring (see [PR #78](https://bitbucket.org/osrf/vrx/pull-requests/78/issue-23-coordinate-the-physics-and/diff) and [Issue #23](https://bitbucket.org/osrf/vrx/issues/23/coordinate-the-physics-and-visualization) ) the VRX environment implements a model of ocean surface waves that synchronizes the visual representation with the physical influence on models within the environment.  This page is intended to explain the implementation.

# Models #

## Ocean Model #

The `wave_gazebo/world_models/ocean_waves` model includes three plugins:

  1. **WavefieldModelPlugin:ModelPlugin** 
    * Includes instances of 
        * WavefieldEntity:gazebo::physics::Base which includes an instance of
            * WaveParameters, defined in Wavefield.hh, which holds the current values to define the wave field (number, angle, scale, etc.)
 and a Wavefield object. Wave field parameters are passed to the Wavefield object. Subscribes to Gazebo message on ~/request. When receives message, responds on ~/response with wave field parameters.
    * Subscribed Gazebo Topics:
        * ~/request ([gazebo::msgs::Request](https://bitbucket.org/osrf/gazebo/src/default/gazebo/msgs/request.proto))
        * ~/wave ([gazebo::msgs::Param_V](https://bitbucket.org/osrf/gazebo/src/default/gazebo/msgs/param_v.proto))  - Allows for setting the values of the WaveParameters instance via gazebo topic.  The WaveMsgPublisher utility is supplied to support.
    * Published Gazebo Topics
        * ~/response ([gazebo::msgs::Response](https://bitbucket.org/osrf/gazebo/src/default/gazebo/msgs/response.proto))
  1. **WavefieldVisualPlugin:VisualPlugin**  Requests wave parameters from the WavefieldModelPlugin via Gazebo message. Uses rendering API to set input parameters to OpenGL shader GernstnerWaves.frag.
  1. A second **WavefieldVisualPlugin:VisualPlugin** for below the water surface. Uses the same parameters and shader.

# Examples

Not sure why the command line tools doesn't work https://bitbucket.org/osrf/gazebo/pull-requests/2907/add-cogazebo::transport::requestmmand-to-send-a-request-with-gz/diff
Perhaps not in Gazebo 7?