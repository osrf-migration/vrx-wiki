#Creating Many Test Worlds#
#Tutorial#
Lets start with a though experiment.

Lets say we have code for a robot written and are able to simulate this robot in
Gazebo. Lets also say we need to test this code on doing 3 tasks in bright and
dark lighting conditions.

We could write all 6 world files COMPLETELY by hand, but that would be tedious,
so lets write them with xacro macros (see xacro tutorial if this is new to you).

But what if in the future it turns out that your robot needs to work in 3
different weather conditions and 3 NOT 2 lighting conditions. Now, to test all these factors independently from one another we need to manually write the xacro macro calls for 27=(3*3*3) worlds!!!

That is far too tedious, if only we had a program to generate all 27 of those
world files!

Good News, we do!

1.Create a directory some where to hold some things ie:

mkdir ~/generated_worlds
cd generated_worlds/

2.Create a yaml according to the yaml filling instructions (also see example):
gedit worlds.yaml

(example as applied to Robot X)
```
########worlds.yaml###########
# We need to decide what is constant between all of our simulations.
# We will call this 'axis' constant
constant:
    # The constant axis will have only one variation on itself, so 1 step.
    steps: 1
    # This is a list of the macros that this axis will contribute
    # to a given world
    macros:
        # For RobotX, the sandisland xacro macro is constant
        # NOTE: to reference a macro here, it's file MUST be included by
        # vrx_gazebo/worlds/xacors/include_all_xacros.xacro
        sandisland2:
        # NOTE: Later on, we use the scene_macro macro, if you do not want
        # to use scene_macro, use normal sandisland macro, NOT sandisland2.
        # sandisland has no parameters, but all macros MUST have a (-) indented
        # on the line below to show that there is at least ONE instance of them
        # (even if it is empty, more on this later).
            -
    # Every axis MUST have a sequence field (even if it is empty)
    # (more on what sequence does later)
    sequence:

# Now lets pick something that will vary independently of all other things we
# want to test between the simulations, say position of the sun (time of day).
# ie: we want to test all tasks at all times of day. Time of day will be
# independent of the task being tested
environment:
    # Lets say we want to test 5 times of day, so 5 steps
    steps: 5
    macros:
        # Name of xacro macro that does time of day
        # (defined in scene_macro.xacro)
        scene_macro:
            # Parameter of time_of_day macro
            # We want the time of day to start at 8am (8) and go three hours
            # between steps to end at 8pm (20)
            # so we want to test times 8, 11, 14, 17, and  20
            # we can express this as a function based on the index of this
            # axis(n)! NOTE: n starts at 0 and goes to steps-1.
            # NOTE: the parameter will be evaluated as a Python lambda.
            # NOTE: all parameters defined under macros of an axis
            # (NOT defined in sequence) will be evaluated as Lambdas
            # (unless the "' '" is used).
            - time: (3*n)+8
    # sequence MUST be here, even if it is empty
    sequence:

# Now lets get those tasks that we originally wanted to test in here
tasks:
    # Lets say we wanted to test 2 tasks, but when we are testing one, we do not
    # want to test the other
    steps: 2
    # so we say that this axis will contribute these macros to the world files,
    # and we fill out the parameters that we want
    # NOTE: these technically do not need to be here do too this axis's
    # application (all steps overridden by sequence), but I think it makes the
    # axis more descriptive
    # NOTE: in order to specify a string evaluated parameter
    # (as opposed to the default: functional) use "'my_string'"
    macros:
        nav_challenge:
            - name: "'nav_challenge'"
        light_buoy:
            - name: "'stc'"
    # Now we learn about sequence
    # sequence is a way to override whatever macros are defined
    sequence:
        # So lets say for step 0, we want to run the nav_challenge
        0:
            # So we include the nav_challenge here and overload ALL
            # its parameters
            # NOTE: When a step is overridden by sequence, the macros are
            # directly passed to the xacro macro, no Lambda evaluation.
            # NOTE: if a macro is excluded in a sequence override on an axis,
            # it will also be excluded from world files of that coordinate of
            # that axis
            # ie: worlds with the 'tasks' coordinate == 0 will NOT have the
            # light_buoy macro, the ONLY macros contributed by this axis for
            # 'tasks' of 0, will be nav_challenge
            nav_challenge:
                - name: "'nav_challenge'"
        # So now we want want to test the light_buoy
        1:
            # So we include the light buoy and specify all the parameters
            # we do not want to be default
            light_buoy:
                - name: "'stc'"

# That is it, so now we will have 10 world xacros and subsequent world files.
# ALL of them will have the sandisland macro
# They will test the nav_challenge at 5 times of the day
# They will test the light_buoy at 5 times of a day
```

#Quick Start Instructions#
1.Create a directory some where to hold some things ie:

`mkdir ~/generated_worlds`
`cd generated_worlds/`

2.Create a yaml according to the yaml filling instructions (also see example):

`gedit worlds.yaml`

3.Make a new directory to hold the world xacros for convenience:

`mkdir world_xacros/`

4.Same for worlds:

`mkdir worlds/`

5.Run the script:

`roslaunch vrx_gazebo generate_worlds.launch requested:=/home/<username>/generated_worlds/worlds.yaml world_xacro_target:=/home/<username>/generated_worlds/world_xacros/ world_target:=/home/<username>/generated_worlds/worlds/ --screen`

6.See the success message: All  <n>  worlds generated

7.Examine the generated xacros under world_xacros/ and make sure they are what
you want (these are meant to be more human readable than the .worlds files):

`gedit world_xacros/world0.world.xacro`

8.Run one of your new worlds:

`roslaunch vrx_gazebo sandisland.launch world:=/home/<username>/generated_worlds/worlds/world0.world`