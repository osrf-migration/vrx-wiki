# WAMV Command Challenge

For the 2019 RobotX Interactive event the challenge is based upon the classic game  [Missile Command](https://en.wikipedia.org/wiki/Missile_Command).  

## Game Overview

The game consists of a series of attackers, where each attacker is a simplified WAM-V craft colored red, green or yellow.  The attackers are attempting to reach the beach is defined by the land-water interface included between the two ground station models

![scenario_annote.png](https://bitbucket.org/repo/BgXLzgM/images/344213614-scenario_annote.png)


The attackers spawn roughly 100 m from the beach and approach the beach with a constant velocity.  You score points by having your WAM-V intercept the attackers.  The WAM-V intercepts an attacker when the distance between the WAM-V and an attacker is less than 5 m.  When an attacker is intercepted it disappears from the scene and 1 point is awarded.  

![Screenshot from 2019-12-01 16-30-24b.png](https://bitbucket.org/repo/BgXLzgM/images/1272893178-Screenshot%20from%202019-12-01%2016-30-24b.png)

The game is over when one of the attackers reaches the beach.  An attacker reaches the beach when it gets within roughly 5 m of the shoreline.

The game consists of a series of levels, starting at level 1.  When you intercept all the attackers in a level the next level begins by respawning attackers at random locations 100 m from the beach.  The number of attackers in each level is equal to the level number (so in level 2 you will have 2 attackers, etc.)  The speed of the attackers also increases with each level.

Here is a [**demonstration video**](https://vimeo.com/user5784414/review/376721424/5a2d3df4eb) illustrating how the number and speed of the attackers increases with each level.  Notice that at the end of the video the attackers are able to make it to the beach and the game is over.   

The game continues as long as you are able to intercept the attackers.  As soon as any attacker reaches the beach the game is over and all the attackers stop in place. 

Here is another [**demonstration scoring video**](https://vimeo.com/user5784414/review/376723116/7d5ebd63e2) to illustrate how intercepting attackers increases the score.

As was done in the VRX challenge, the score is published as a [vrx_gazebo Task](https://bitbucket.org/osrf/vrx/src/default/vrx_gazebo/msg/Task.msg) message on the `/vrx/task/info` topic.  The following fields of the Task message are populated:

 * `name` is always "wamv_command".
 * `state` is either "running" if the attackers have not yet reached the beach and then "finished" once an attacker has reached the beach.
 * `score` is the current numerical score. 

### Objectives

 * Generate a high score and report it on the shared document (honor system)
 * Describe your solution
 * Submit a highlight video

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

A game is run by the wamv_commander node.  Once the simulation is up you can start the node with
```
rosrun wamv_command wamv_command.py 
```

You can monitor your score and the game state with
```
rostopic echo /vrx/task/info
```

Teloperation using a gamepad.
```
roslaunch vrx_gazebo usv_joydrive.launch 
```

## Solution Progression

1. Teleoperation
2. Single defender
3. Team defenders

## Observing Attacker/Defender State

The state of all of the models in the Gazebo world are published on on the ROS topic `/gazebo/model_states`.  This provides the true state of each model in the gazebo coordinate frame.  For WAMV Command challenge we will use the ground truth so that you system has complete, error-free knowledge of the defender and attacker states.

We will provide an example of subscribing to this ROS topic for state information.


## Expanding to Multiple Defenders

Based on mutltivehicle tutorial.