# ROS_TP

The packages are structured according to what I have found in different repos with similar functionality.

gripx_control contains the ros_controllers

gripx_description contains the urdf files, meshes and some rviz configs
The currently used urdf-file is callded: gripx_gazebo

gripx_gazebo contains a launch file to only launch gazebo with the newest urdf.

grips_moveit_config is the autogenerated package from moveit.
Some files have been manually added:
- controllers.yaml
- ros_controllers.yaml (Manually added lines 24-44)
- moveit_planning_execution.launch

moveit_packages: The implementation of the different controllers to send trajectory commands.
-->controllers: Here i tried to split up every controller according to the type
The rest of the packages are basically the tutorials from moveit, which adapted a little.

marker1: A small package to spawn some markers in rviz. This one works, but the topic to which the message is sent has to be updated.

To spawn the robot and use the controllers:

    Spawn the robot in gazebo:

roslaunch gripx_gazebo gazebo.launch

    Load the moveit environment and rviz (opens 2 rviz for the moment)

roslaunch gripx_moveit_config moveit_planning_execution.launch

In case the motion planning gui does not show up, simply add it in rviz.

3)Launch the different controllers with: roslaunch moveit_packages cartesian_control.launch roslaunch moveit_packages pose_control.launch roslaunch moveit_package position_control.launch roslaunch moveit_package position_control_with_callback.launch

The goal until next friday woud be, to adapt the controllers in such a way, that they subscribe to a topic where the end positions are published to (same as used in the marker1 package).

After receiving the coordinates, the path planning should be split in 3 parts: Part 1: Posecontrol to the location but -0.1m in z-direction. Part 2: Cartesian control along the z axis, 0.1m Part 3: Move the arm to a predefined deploy location

pose_control.cpp as functionality 1 and 2 included, but i am currently struggling with the right quaternion orientation.









