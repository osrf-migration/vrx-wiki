# Creating Many Test Worlds

# Tutorial

Lets start with a thought experiment.

Let's say we have code for a robot ready and we are able to simulate this robot in
Gazebo. Let's also say we need to test this code's performance on 3 tasks, both in bright and
dark lighting conditions.

We could write all 6 world files COMPLETELY by hand, but that would be tedious,
so let's write them with xacro macros (see xacro tutorial if this is new to you).

But what if in the future it turns out that your robot needs to work in 3
different weather conditions and 3 NOT 2 lighting conditions. Now, to test all these factors independently from one another, we would need to manually write the xacro macro calls for 27 worlds!!!

That is far too tedious! If only we had a program to generate all 27 of those
world files!

Good News, we do!

# Quick Start Instructions

1.Create a directory to hold store our files ie:

`mkdir ~/generated_worlds`

`cd ~/generated_worlds`

2.Create a yaml (copy RobotX Nav Challenge Example below):

`gedit ~/generated_worlds/worlds.yaml`

3.Make a new directory to hold the world xacros for convenience:

`mkdir ~/generated_worlds/world_xacros/`

4.Same for worlds:

`mkdir ~/generated_worlds/worlds/`

5.Run the script (assuming `$HOME=/home/<username>`):

`roslaunch vrx_gazebo generate_worlds.launch requested:=$HOME/generated_worlds/worlds.yaml world_xacro_target:=$HOME/generated_worlds/world_xacros/ world_target:=$HOME/generated_worlds/worlds/ --screen`


6.See the success message:

```
All  8  worlds generated
================================================================================REQUIRED process [world_gen-2] has died!
process has finished cleanly
log file: /home/tylerlum/.ros/log/ab466cb2-c83b-11e9-a434-dcfb48e97aeb/world_gen-2*.log
Initiating shutdown!
================================================================================
```

7.Examine the generated xacros under world_xacros/ and make sure they are what
you want (these are meant to be more human readable than the .worlds files):

`gedit ~/generated_worlds/world_xacros/worlds0.world.xacro`


8.Run one of your new worlds:


`roslaunch vrx_gazebo sandisland.launch world:=$HOME/generated_worlds/worlds/worlds0.world`

# RobotX Nav Challenge (Aug 2019)

```
constant:
    steps: 1
    macros:
        sandisland_minus_scene: 
            -
    sequence:

tasks:
    steps: 1
    macros:
        nav_challenge:
            -
    sequence:
        0:
            nav_challenge:
                - name: nav_challenge
                  uri: navigation_course
                  /**gates: "
                  <gate>
                    <left_marker>red_bound_0</left_marker>
                    <right_marker>green_bound_0</right_marker>
                  </gate>
                  <gate>
                    <left_marker>red_bound_1</left_marker>
                    <right_marker>green_bound_1</right_marker>
                  </gate>
                  <gate>
                    <left_marker>red_bound_2</left_marker>
                    <right_marker>green_bound_2</right_marker>
                  </gate>
                  <gate>
                    <left_marker>red_bound_3</left_marker>
                    <right_marker>green_bound_3</right_marker>
                  </gate>
                  <gate>
                    <left_marker>red_bound_4</left_marker>
                    <right_marker>green_bound_4</right_marker>
                  </gate>
                  <gate>
                    <left_marker>red_bound_5</left_marker>
                    <right_marker>green_bound_5</right_marker>
                  </gate>
                  <gate>
                    <left_marker>red_bound_6</left_marker>
                    <right_marker>green_bound_6</right_marker>
                  </gate>"
waves:
    steps: 2
    macros:
        ocean_waves:
            -
    sequence:
        0:
            ocean_waves:
                - gain: 1.0
                  period: 1.0
        1:
            ocean_waves:
                - gain: 0.4
                  period: 8.0
wind:
    steps: 2
    macros:
        usv_wind_gazebo:
            -
    sequence:
        0:
            usv_wind_gazebo:
                - mean_vel: 0.0
                  var_gain: 0
                  var_time: 2
                  seed: 10
                  /**wind_objs: "
                  <wind_obj>

                  <name>wamv</name>

                  <link_name>base_link</link_name>

                  <coeff_vector>.5 .5 .33</coeff_vector>

                  </wind_obj>"
        1:
            usv_wind_gazebo:
                - mean_vel: 8.0
                  var_gain: 8.0
                  var_time: 20
                  seed: 10
                  /**wind_objs: "
                  <wind_obj>

                  <name>wamv</name>

                  <link_name>base_link</link_name>

                  <coeff_vector> .5 .5 .33</coeff_vector>

                  </wind_obj>"
scene:
    steps: 2
    macros:
        scene_macro:
            -
    sequence:
        0:
            scene_macro:
                - /**fog: "
                  <color> 0.7 0.7 0.7 1 </color>

                  <density> 0.0 </density>"
                  /**ambient: "1 1 1 1"
        1:
            scene_macro:
                - /**fog: "
                  <color> 0.9 0.9 0.9 1 </color>

                  <density> 0.1 </density>"
                  /**ambient: "0.3 0.3 0.3 1"


```

# RobotX Example 

Note: this example is not currently working. It contains more detailed descriptions of the various parts of the world yaml.

You can view working examples [here](https://bitbucket.org/osrf/vrx/src/default/vrx_gazebo/worlds/yamls/)

```
########worlds.yaml###########
#we need to decide what is constant between all of our simulations. We will call this 'axis' constant
constant:
    #the constant axis will have only one variation on itself, so 1 step.
    steps: 1
    #this is a list of the macros that this axis will contribute to a given world
    macros:
        #for RobotX, the sandisland xacro macro is constant
        #NOTE: to reference a macro here, it's file MUST be included by vrx_gazebo/worlds/xacors/include_all_xacros.xacro
        sandisland:
        #sandisland has no parameters, but all macros MUST have a (-) indented on the line below to show that there is at least ONE instance of them (even if it is empty, more on this later)
            -
    #every axis MUST have a sequence field (even if it is empty)(more on what sequence does later)
    sequence:

#now lets pick something that will vary independently of all other things we want to test between the simulations, say strength of the wind 
environment:
    #lets say we want to test 5 strengths of wind, so 5 steps
    steps: 5
    macros:
        #name of xacro macro that does time of day(defined in scene_macro.xacro)
        usv_wind_gazebo:
            #parameter of time_of_day macro 
            #we want the time of day to start at 8am(8) and go three hours between steps to end at 8pm(20)
            # so we want to test times 8, 11, 14, 17, and  20
            #we can express this as a function based on the index of this axis(n)! NOTE: n starts at 0 and goes to steps-1.
            #NOTE: in order to specify a string is an evaluated parameter (as opposed to the default: functional) use "'<my_string>'"
            #NOTE: this parameter will always evaluate to the same string(except when a sequence override is present)
        - /**wind_objs: "'
          <wind_obj>

            <name>wamv</name>
          
            <link_name>base_link</link_name>
          
            <coeff_vector>.5 .5 .33</coeff_vector>
          
          </wind_obj>'"
          #NOTE: the parameter mean_vel will be evaluated as a python lambda.
          mean_vel: 2*(n+1)
    #sequence MUST be here, even if it is empty)
    sequence:

#now lets get those tasks that we originally wanted to test in here 
tasks:
    #lets say we wanted to test 2 tasks, but when we are testing one, we do not want to test the other
    steps: 2
    #so we say that this axis will contribute these macros to the world files, and we fill out the parameters that we want
    #NOTE: these actually do not need to be here due too this axis's application, but I think it makes the axis more descriptive
    macros:
        nav_challenge:
            - name: "'nav_challenge1'"
        light_buoy:
            - name: "'stc'"
    #now we learn about sequence
    #sequence is a way to override whatever macros are defined 
    sequence:
        #so lets say for step 0, we want to run the nav_challenge
        0:
            #so we include the nav_challenge here and overload ALL of its parameters
            #NOTE: when a step is overriden by sequence, the macros are directly passed to the xacro macro, no Lambda evaluation
            #NOTE: if a macro is excluded in a sequence-override on an axis, it will also be excluded from world files of that coordinate of that axis
            #ie: worlds with the 'tasks' coordinate == 0 will NOT have the light_buoy macro, the ONLY macros contributed by this axis for 'tasks'  0, will be nav_challenge
            nav_challenge:
                - name: "'nav_challenge'"
        #so now we want want to test the light_buoy
        1:
            #so we include the light buoy and overload ALL its parameters
            light_buoy:
                - name: "'stc'"

#That is it, so now we will have 10 world xcaros and subsiquent world files.
#ALL of them will have the sandisland macro
#They will test the nav_challenge at 5 times of the day
#They will test the light_buoy at 5 times of a day


```