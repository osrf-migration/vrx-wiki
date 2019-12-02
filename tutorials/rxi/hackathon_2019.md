# WAMV Command Challenge

For the 2019 RobotX Interactive event the challenge is based upon the classic game  [Missile Command](https://en.wikipedia.org/wiki/Missile_Command).  

## Game Overview

The game consists of a series of attackers, where each attacker is a simplified WAM-V craft colored red, green or yellow.  The attackers are attempting to reach the beach is defined by the land-water interface included between the two ground station models

![scenario_annote.png](https://bitbucket.org/repo/BgXLzgM/images/344213614-scenario_annote.png)


The attackers spawn roughly 100 m from the beach and approach the beach with a constant velocity.  You score points by having your WAM-V intercept the attackers.  The WAM-V intercepts an attacker when the distance between the WAM-V and an attacker is less than 5 m.  When an attacker is intercepted it disappears from the scene and 1 point is awarded.  

![Screenshot from 2019-12-01 16-30-24b.png](https://bitbucket.org/repo/BgXLzgM/images/1272893178-Screenshot%20from%202019-12-01%2016-30-24b.png)




The game is over when one of the attackers reaches the beach.  An attacker reaches the beach when it gets within roughly 5 m of the shoreline.

The game consists of a series of levels, starting at level 1.  When you intercept all the attackers in a level the next level begins by respawning attackers at random locations 100 m from the beach.  The number of attackers in each level is equal to the level number (so in level 2 you will have 2 attackers, etc.)  The speed of the attackers also increases with each level.

## Getting Started

The following instructions assume you have a functioning VRX simulation evironment working on your machine.

### Setup

Clone the wamv_command git repository.  This should be in the same ROS workspace as the VRX force code, e.g., `~/vrx_ws/src`
```
cd ~/vrx_ws/src
git clone git@bitbucket.org:brian_bingham/wamv_command.git
```

Build the ROS package with `catkin_make`

### Execution

First we will launch the simulation environment with a single WAM-V USV in the world.

```
roslaunch wamv_command wamv_command.launch verbose:=true
```