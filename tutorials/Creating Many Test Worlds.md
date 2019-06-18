########worlds.yaml###########
#we need to decide what is constant between all of our simulations. We will call this 'axis' constant
constant:
    #the constant axis will have only one variation on itself, so 1 step.
    steps: 1
    #this is a list of the macros that this axis will contribute to a given world
    macros:
        #for RobotX, the sandilsand xacro macro is constant
        #NOTE: to refence a macro here, it's file MUST be included by vrx_gazebo/worlds/xacors/include_all_xacros.xacro
        sandisland:
        #sandisland has no parameters, but all macros MUST have a (-) indented on the line below to show that there is at least ONE instance of them(even if it is empty, more on this later)
            -
    #every axis MUST have a sequence feild (even if it is empty)(more on what sequence does later)
    sequence:

#now lets pick soething that will varry independently of all other things we want to test between the simulations, say strength of the wind 
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
            #NOTE: in order to specify a string evaluated parameterter (as opposed to the default: functional) use "'<my_string>'"
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
    #NOTE: these actually do not need to be here do too this axis's application, but I think it makes the axis more decriptive
    macros:
        nav_challenge:
            - name: "'nav_challenge1'"
        light_buoy:
            - name: "'stc'"
    #now we learn about sequence
    #sequence is a way to override whatever macros are dinfined 
    sequence:
        #so lets say for step 0, we want to run the nav_challenge
        0:
            #so we include the nav_chanllenge here and overload ALL it parameters
            #NOTE: when a step is overriden by sequence, the macros are directly passed to the xacro macro, no Lambda evaluation
            #NOTE: if a macro is excluded in a sequence override on an axis, it will also be excluded from world files of that coordinate of that axis
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