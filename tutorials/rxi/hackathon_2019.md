[TOC]

# WAMV Command Challenge

For the 2019 RobotX Interactive event the challenge is based upon the classic game  [Missile Command](https://en.wikipedia.org/wiki/Missile_Command).  

## Game Overview

The game consists of a series of attackers, where each attacker is a simplified WAM-V craft colored red, green or yellow.  The attackers are attempting to reach the beach is defined by the land-water interface included between the two ground station models as shown in the image below.

![scenario_annote.png](https://bitbucket.org/repo/BgXLzgM/images/344213614-scenario_annote.png)

The attackers spawn roughly 75 m from the beach and approach the beach with a constant velocity.  You score points by having your WAM-V intercept the attackers.  The WAM-V intercepts an attacker when the distance between the WAM-V and an attacker is less than 5 m.  When an attacker is intercepted it disappears from the scene and 1 point is awarded.  

![Screenshot from 2019-12-01 16-30-24b.png](https://bitbucket.org/repo/BgXLzgM/images/1272893178-Screenshot%20from%202019-12-01%2016-30-24b.png)

The game is over when one of the attackers reaches the beach.  An attacker reaches the beach when it gets within roughly 5 m of the shoreline.

The game consists of a series of levels, starting at level 1.  When you intercept all the attackers in a leve,  the next level begins by respawning attackers at random locations 100 m from the beach.  The number of attackers in each level is equal to the level number (so in level 2 you will have 2 attackers, etc.)  The speed of the attackers also increases with each level.

### Demonstration Videos

* [**WAMV Command Demo**](https://vimeo.com/376721424) illustrating how the number and speed of the attackers increases with each level.  Notice that at the end of the video the attackers are able to make it to the beach and the game is over.   The game continues as long as you are able to intercept the attackers.  As soon as any attacker reaches the beach the game is over and all the attackers stop in place. 

* [**WAMV Command Scoring Demo**](https://vimeo.com/376723116) showing how intercepting attackers increases the score.  In the video the defender is being teleoperated.

* [**WAMV Command: Two Defenders**](https://vimeo.com/377202478) illustrating two demonstration modes with two WAMV defenders.

### Interfaces and Customization

* Teams must interact with the WAM-V through the propulsion interface, i.e., by issuing thruster commands.
* Teams are allowed to modify the `wamv_command.launch` file to spawn multiple USVs (if collaborating with other teams) and to change the propulsion configuration of the USVs.
* As was done in the VRX challenge, the score is published as a [vrx_gazebo Task](https://bitbucket.org/osrf/vrx/src/default/vrx_gazebo/msg/Task.msg) message on the `/vrx/task/info` topic.  The following fields of the Task message are populated:
    * `name` is always "wamv_command".
    * `state` is either "running" if the attackers have not yet reached the beach and then "finished" once an attacker has reached the beach.
    * `score` is the current numerical score.

## Collaboration

Teams are encouraged to work together to mazimize their performance.  A single team can only use one defender WAM-V during game play.  If multiple teams collaborate they can use multiple defender USVs to intercept attackers, providing an advantage for teams that can collaborate on joint solutions.

Groups are allowed to modify the `wamv_command.launch` file to enable spawning of multiple USVs - see the [Multiple Vehicle Tutorial](https://bitbucket.org/osrf/vrx/wiki/tutorials/rxi/multivehicle) for guidance on simulating multiple USVs simultaneously.

This event is purposely termed a "challenge" as opposed to a "competition".  Teams are also encouraged to share their solutions with other teams.  Hosting code in an online version control repository (e.g., github, bitbucket, etc.) is a convenient way to share solutions.

## Challenge Categories

The following types of soluitons are anticipated and each type will be consider in its own category:

* **Single-Teleop** The simplest category where a team uses a single USV and teleoperates the USV to intercept attackers.
* **Multi-Teleop** A collaboration of teams to spawn two (or more) USVs and teleoperate the USVs to intercept attackers.
* **Single-Pursuit** A single team uses a single USV and programmatically guides the USV to intercept the attackers.
* **Multi-Pursuit** A collaboration of teams to spawn two (or more) USVs that are programmatically guided to intercept the attackers

Teams may enter scores in multiple categories - it may make sense to start simple and begin collaborating and experimenting as the solutions evolve. 

## Multiple Objectives

Your performance in the RXI Challenge is more than just the game score.  The purpose of the event is to generate innovate solutions and share those solutions with the broader community.  The desired end-state of the event is to have a number teams working together to demonstrate their solutions.

#### High scores

Generate a high score and report it on the [shared leaderboard](https://docs.google.com/spreadsheets/d/1fvYe2w-jMXbQlqDLx7OO0jIWo6Y0uX9KVVEgFyEDzQI/edit?usp=sharing).

#### Describe your solution

Share a description of your solution by posting a few slides describing the approach.  Share the slides by posting a link on the [shared leaderboard](https://docs.google.com/spreadsheets/d/1fvYe2w-jMXbQlqDLx7OO0jIWo6Y0uX9KVVEgFyEDzQI/edit?usp=sharing).  Time will be made at the end of the event to discuss some of the innovative solutions.

#### Generate a highlight video

A concise way to convey your performance is to capture a video of the simulation running to highlight various aspects of the solution.  Include a link to your highlight video in the [shared leaderboard](https://docs.google.com/spreadsheets/d/1fvYe2w-jMXbQlqDLx7OO0jIWo6Y0uX9KVVEgFyEDzQI/edit?usp=sharing).

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

## Observing Attacker/Defender State

* All attacker USVs have the string 'attacker' in their model name.
* If you create new USV models for use as defenders, make sure that the model name includes the string 'defender'.  The game looks for any USVs with that name to determine interceptions. 

The state of all of the models in the Gazebo world are published on on the ROS topic `/gazebo/model_states`.  This provides the true state of each model in the gazebo coordinate frame.  For WAMV Command challenge we will use the ground truth so that you system has complete, error-free knowledge of the defender and attacker states.

You will find that when the attackers are intercepted they are not deleted from the world, instead they are simply moved out of view.  (There seems to be some issues using the ROS interface to delete models.)  Models that are intercepted are moved to x=10,000, y=10,000 and have a speed=0.0.  If you are programmatically looking for attackers

The RXI ROS package includes the `nodes/state_observer_ex.py` program to illustrate one method for using the gazebo model states publication to programmatically determine the pose and velocity of all the defenders and attackers.


## Expanding to Multiple Defenders

If you are able to get a single defender USV working, we encourage you to join forces with another team (or more) to employ multiple defenders.  Following the instructions from the [Multiple Vehicles](https://bitbucket.org/osrf/vrx/wiki/tutorials/rxi/multivehicle) tutorial, a two USV launch file is included in the wamv_command package:
```
roslaunch wamv_command wamv_command_multi.launch 
```