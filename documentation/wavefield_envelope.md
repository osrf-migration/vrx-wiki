# Wavefield Envelope #

Our goal is to define the simulated wave field so that scenarios reflect the full range of conditions that may be encountered in the physical RobotX competition and are within capabilities of the VRX simulation.

## Wavefield Specification ##

For VRX we use the Pierson-Moskowitz wave spectrum sampling (PMS) model described in detail in the documentation (will show up once we merge the wave_visualization branch).  The parameters of the wavefield model are user-specified within the ocean_waves model definition as SDF parameters for the WavefieldModelPlugin.  For example:

```
<wave>
  <model>PMS</model>
  <number>3</number>
  <scale>1.5</scale>
  <steepness>0.0</steepness>
  <amplitude>0.1</amplitude>  <!-- No effect for the PMS model -->
  <period>2.0</period>
  <gain>1.0</gain>
  <direction>1.0 0.0</direction>
  <angle>0.4</angle>				
  <tau>1.0</tau>
</wave>
```

The only parameters that will be varied are period, gain, direction and angle.  The direction and angle parameters control the horizontal angles of the constituent waves and can vary over the full 360 degrees. 

## Period and Gain Envelope ##

The range of allowable period and gain parameter limits were selected through empirical testing of the simulation to find suitable values to challenge the participants while remaining relevant to RobotX conditions.

The testing setup is a custom ocean world with two WAM-V's (with cameras), a few of the markers and wavegauge (more on that below).
```
roslaunch wave_gazebo wave_wamv.launch verbose:=true paused:=true
```
which should result in a view similar to this.
![gz_wavefield_test_mr.png](https://bitbucket.org/repo/BgXLzgM/images/695865771-gz_wavefield_test_mr.png)

Based on running the simulation in a variety of conditions, we arrive at the following envelope of acceptable wave field parameters for VRX evaluation.

![wavefield_envelope.png](https://bitbucket.org/repo/BgXLzgM/images/2265349497-wavefield_envelope.png)  

## Beyond Period and Gain Limits ##