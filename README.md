# learning_moveit_jaco

## Moveit! integration with Kinova Jaco-2 arm.

+ You should complete the MoveIt! Setup Assistant tutorial and have a MoveIt! configuration for the JACO that you can use.

+ Here is a link to configure Moveit! for JACO.
https://github.com/JenniferBuehler/jaco-arm-pkgs/wiki/Setup-Jaco-with-MoveIt

+ The generated MoveIt! configuration package in our case is called “jaco_moveit_config”. Load the jaco_robot.urdf.xacro file from the jaco_description package to do this.

## Notes
+ In Motion Plannig Tutorials added the launch of jaco_moveit_config/launch/move_group.launch in the motion_plannig_api_tutorial.launch. Was not in pr2_tutorial but it still worked. Dont know why!?

## Useful Links (for debugging)
+ http://answers.ros.org/question/84806/using-moveit-to-actually-control-a-robot/
+ https://groups.google.com/forum/#!topic/moveit-users/qAi4p3HAp1c
+ https://groups.google.com/forum/#!topic/moveit-users/4CXQhs2BlJg
+ https://groups.google.com/forum/#!topic/moveit-users/R9Fu2bmc_2M
+ http://answers.ros.org/question/192739/implement-moveit-on-real-robot/
