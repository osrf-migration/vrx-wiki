![gui_screen_shot.png](https://bitbucket.org/repo/BgXLzgM/images/2361812394-gui_screen_shot.png)

This is a handy gui overlay to help debug issues right in gazebo. On the left is task information. In the middle is wamv heading (in blue) and wind direction/magnitude(in red), and on the right is a contact indicator that lights up whenever the wamv is in contact with an object.

Note: Inorder to have full functionality, a scoring plugin and the wind plugin must be included in the world and the environmental variable: VRX_DEBUG must not equal "false".

Note: This plugin uses the /vrx/debug ros topics to function. In competition, THESE WILL NOT BE AVAILABLE.

#Compiling from source:
If you compiled vrx from source:

1. find src/vrx/vrx_gazebo/worlds/sandisland.xacro

2. uncomment line 35 of the file so it looks like
`<plugin name="GUITaskWidget" filename="libgui_task_widget.so"/>`

3. recompile vrx with catkin_make in order to regenerate the worlds from the xacro files.

4. Now, all worlds that use the sandisland.xacro (every world in vrx repo) will have the gui plugin enabled.

5. Test to make sure everything works by running
`roslaunch vrx_gazebo navigation_task.launch`

6. You should see an overly that looks like the one in the screen shot above

#From Packages
If downloaded vrx from a package manager:

1. Find the directory vrx_gazebo/worlds/

(you can use:
`find / | grep vrx_gazebo/worlds/navigation_task.world`)

2. Open file navigation_task.world

3. Find the gui block (should be around line 32) and look like:
`<gui fullscreen="0">`

4. uncomment the line that looks like:
`<!--<plugin name="GUITaskWidget" filename="libgui_task_widget.so"/>-->`
so that it looks like 
`<plugin name="GUITaskWidget" filename="libgui_task_widget.so"/>`

5. Test to make sure everything works by running
`roslaunch vrx_gazebo navigation_task.launch`

6. You should see an overly that looks like the one in the screen shot above