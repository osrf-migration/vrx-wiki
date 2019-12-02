# RXI Tutorial: Operating Multivehicles

Operating more than one robot can be challenging, but using namespaces and remapping can help when working with multiple robots.  In this tutorial we will spawn two WAM-V robots within a Gazebo simulation and demonstrate how namespaces can be used to teleoperate the two robots independently.  If time allows, we will work on a leader-follower configuration where a follower WAM-V automatically drives toward the teleoperated leader robot.  

## Resources ##

* [ROS Names](http://wiki.ros.org/Names) documentation
* [A Gentle Introduction to ROS](https://www.cse.sc.edu/~jokane/agitr/) by Jason M. O'Kane, Chapter 5: Graph Resource Names.
* Roslaunch [<group> tag](http://wiki.ros.org/roslaunch/XML/group) - notice the namespace (ns) attribute