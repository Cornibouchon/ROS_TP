#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
// Used for the convertion from euler to quaternion see: http://wiki.ros.org/tf2/Tutorials/Quaternions
#include <tf2/LinearMath/Quaternion.h>
// Used to  transform between tf2_quaternion and geometry_msgs quaternion
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Setup
  // ^^^^^
  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
  // are used interchangably.
  //static const std::string PLANNING_GROUP = "panda_arm";
  static const std::string PLANNING_GROUP = "gripx_arm";

  // The :planning_interface:`MoveGroupInterface` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // We will use the :planning_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Visualization
  // ^^^^^^^^^^^^^
  //
  // The package MoveItVisualTools provides many capabilities for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script.
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // Quaternion transformation
  tf2::Quaternion myQuaternion;
  myQuaternion.setRPY( -1.5707, 0, 1.5707 );  // Create this quaternion from roll/pitch/yaw (in radians)
  ROS_INFO_STREAM(myQuaternion);  // Print the quaternion components (0,0,0,1)
  // Normalize to 1
  myQuaternion.normalize();

  move_group.setPositionTarget( 0.5,  0.5,  0.5, move_group.getEndEffectorLink());
  move_group.move();
  move_group.setPositionTarget( 0.5,  0.5,  0.6, move_group.getEndEffectorLink());
  move_group.move();
  move_group.setPositionTarget( 0.5,  0.5,  0.7, move_group.getEndEffectorLink());
  move_group.move();

  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.x = myQuaternion[0];
  target_pose1.orientation.y = myQuaternion[1];
  target_pose1.orientation.z = myQuaternion[2];
  target_pose1.orientation.w = myQuaternion[3]; // move_group.setPlanningTime(10.0);
  target_pose1.position.x = 0.4;
  target_pose1.position.y = 0.1;
  target_pose1.position.z = 0.4;
  move_group.setGoalOrientationTolerance(0.5);
  move_group.setPoseTarget(target_pose1);
   move_group.setPlanningTime(15.0);

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);


  // Moving to a pose goal
  // ^^^^^^^^^^^^^^^^^^^^^
  //
  // Moving to a pose goal is similar to the step above
  // except we now use the move() function. 
  /* Uncomment below line when working with a real robot */
   move_group.move(); 
   ROS_INFO_NAMED("Did it move?", "I hope so:"); 
   visual_tools.trigger();

   // Pose goal 2
  // geometry_msgs::Pose target_pose1;
  for (int i=1;i<=30; i++){
    target_pose1.orientation.x = myQuaternion[0];
    target_pose1.orientation.y = myQuaternion[1];
    target_pose1.orientation.z = myQuaternion[2];
    target_pose1.orientation.w = myQuaternion[3];
    target_pose1.position.x = 0.2;
    target_pose1.position.y = 00;
    target_pose1.position.z = i/50 + 0.3;
    move_group.setGoalTolerance(0.5);
    move_group.setPoseTarget(target_pose1);
    move_group.move(); 
    ROS_INFO_NAMED("Did it move?", "I hope so:"); 
    visual_tools.trigger();
   }
  

  // // Cartesian Paths
  // // ^^^^^^^^^^^^^^^
  // // You can plan a Cartesian path directly by specifying a list of waypoints
  // // for the end-effector to go through. Note that we are starting
  // // from the new start state above.  The initial pose (start state) does not
  // // need to be added to the waypoint list but adding it can help with visualizations
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(target_pose1);

  geometry_msgs::Pose target_pose3 = target_pose1;

  target_pose3.position.z += 0.4;
  waypoints.push_back(target_pose3);  // up

  target_pose3.position.y += 0.4;
  waypoints.push_back(target_pose3);  // up

  target_pose3.position.z += 0.4;
  // target_pose3.position.y += 0.2;
  // target_pose3.position.x -= 0.2;
  waypoints.push_back(target_pose3);  // up and left

  move_group.setMaxVelocityScalingFactor(0.1);

  // // We want the Cartesian path to be interpolated at a resolution of 1 cm
  // // which is why we will specify 0.01 as the max step in Cartesian
  // // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
  // // Warning - disabling the jump threshold while operating real hardware can cause
  // // large unpredictable motions of redundant joints and could be a safety issue
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

  // // Visualize the plan in RViz
  // visual_tools.deleteAllMarkers();
  // visual_tools.publishText(text_pose, "Cartesian Path", rvt::WHITE, rvt::XLARGE);
  // visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  // for (std::size_t i = 0; i < waypoints.size(); ++i)
  //   visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
  // visual_tools.trigger();
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // // Cartesian motions should often be slow, e.g. when approaching objects. The speed of cartesian
  // // plans cannot currently be set  through the maxVelocityScalingFactor, but requires you to time
  // // the trajectory manually, as described `here <https://groups.google.com/forum/#!topic/moveit-users/MOoFxy2exT4>`_.
  // // Pull requests are welcome.
  // //
  // // You can execute a trajectory like this.
  move_group.execute(trajectory);
  // move_group.move(); 

  // // Adding objects to the environment
  // // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // //
  // // First let's plan to another simple goal with no objects in the way.
  move_group.setStartState(*move_group.getCurrentState());
  geometry_msgs::Pose another_pose;
  another_pose.orientation.x = 1.0;
  another_pose.position.x = 0.7;
  another_pose.position.y = 0.0;
  another_pose.position.z = 0.59;
  move_group.setPoseTarget(another_pose);
  move_group.move();

  // success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  // ROS_INFO_NAMED("tutorial", "Visualizing plan 5 (with no obstacles) %s", success ? "" : "FAILED");

  // visual_tools.deleteAllMarkers();
  // visual_tools.publishText(text_pose, "Clear Goal", rvt::WHITE, rvt::XLARGE);
  // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  // visual_tools.trigger();
  // visual_tools.prompt("next step");

  // // The result may look like this:
  // //
  // // .. image:: ./move_group_interface_tutorial_clear_path.gif
  // //    :alt: animation showing the arm moving relatively straight toward the goal
  // //
  // // Now let's define a collision object ROS message for the robot to avoid.
  // moveit_msgs::CollisionObject collision_object;
  // collision_object.header.frame_id = move_group.getPlanningFrame();

  // // The id of the object is used to identify it.
  // collision_object.id = "box1";

  // // Define a box to add to the world.
  // shape_msgs::SolidPrimitive primitive;
  // primitive.type = primitive.BOX;
  // primitive.dimensions.resize(3);
  // primitive.dimensions[primitive.BOX_X] = 0.1;
  // primitive.dimensions[primitive.BOX_Y] = 1.5;
  // primitive.dimensions[primitive.BOX_Z] = 0.5;

  // // Define a pose for the box (specified relative to frame_id)
  // geometry_msgs::Pose box_pose;
  // box_pose.orientation.w = 1.0;
  // box_pose.position.x = 0.5;
  // box_pose.position.y = 0.0;
  // box_pose.position.z = 0.25;

  // collision_object.primitives.push_back(primitive);
  // collision_object.primitive_poses.push_back(box_pose);
  // collision_object.operation = collision_object.ADD;

  // std::vector<moveit_msgs::CollisionObject> collision_objects;
  // collision_objects.push_back(collision_object);

  // // Now, let's add the collision object into the world
  // // (using a vector that could contain additional objects)
  // ROS_INFO_NAMED("tutorial", "Add an object into the world");
  // planning_scene_interface.addCollisionObjects(collision_objects);

  // // Show text in RViz of status and wait for MoveGroup to receive and process the collision object message
  // visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
  // visual_tools.trigger();
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");

  // // Now when we plan a trajectory it will avoid the obstacle
  // success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  // ROS_INFO_NAMED("tutorial", "Visualizing plan 6 (pose goal move around cuboid) %s", success ? "" : "FAILED");
  // visual_tools.publishText(text_pose, "Obstacle Goal", rvt::WHITE, rvt::XLARGE);
  // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  // visual_tools.trigger();
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the plan is complete");

  // // The result may look like this:
  // //
  // // .. image:: ./move_group_interface_tutorial_avoid_path.gif
  // //    :alt: animation showing the arm moving avoiding the new obstacle
  // //
  // // Attaching objects to the robot
  // // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // //
  // // You can attach objects to the robot, so that it moves with the robot geometry.
  // // This simulates picking up the object for the purpose of manipulating it.
  // // The motion planning should avoid collisions between the two objects as well.
  // moveit_msgs::CollisionObject object_to_attach;
  // object_to_attach.id = "cylinder1";

  // shape_msgs::SolidPrimitive cylinder_primitive;
  // cylinder_primitive.type = primitive.CYLINDER;
  // cylinder_primitive.dimensions.resize(2);
  // cylinder_primitive.dimensions[primitive.CYLINDER_HEIGHT] = 0.20;
  // cylinder_primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.04;

  // // We define the frame/pose for this cylinder so that it appears in the gripper
  // object_to_attach.header.frame_id = move_group.getEndEffectorLink();
  // geometry_msgs::Pose grab_pose;
  // grab_pose.orientation.w = 1.0;
  // grab_pose.position.z = 0.2;

  // // First, we add the object to the world (without using a vector)
  // object_to_attach.primitives.push_back(cylinder_primitive);
  // object_to_attach.primitive_poses.push_back(grab_pose);
  // object_to_attach.operation = object_to_attach.ADD;
  // planning_scene_interface.applyCollisionObject(object_to_attach);

  // // Then, we "attach" the object to the robot. It uses the frame_id to determine which robot link it is attached to.
  // // You could also use applyAttachedCollisionObject to attach an object to the robot directly.
  // ROS_INFO_NAMED("tutorial", "Attach the object to the robot");
  // move_group.attachObject(object_to_attach.id, "gripper");

  // visual_tools.publishText(text_pose, "Object attached to robot", rvt::WHITE, rvt::XLARGE);
  // visual_tools.trigger();

  // /* Wait for MoveGroup to receive and process the attached collision object message */
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the new object is attached to the robot");

  // // Replan, but now with the object in hand.
  // move_group.setStartStateToCurrentState();
  // success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  // ROS_INFO_NAMED("tutorial", "Visualizing plan 7 (move around cuboid with cylinder) %s", success ? "" : "FAILED");
  // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  // visual_tools.trigger();
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the plan is complete");

  // // The result may look something like this:
  // //
  // // .. image:: ./move_group_interface_tutorial_attached_object.gif
  // //    :alt: animation showing the arm moving differently once the object is attached
  // //
  // // Detaching and Removing Objects
  // // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // //
  // // Now, let's detach the cylinder from the robot's gripper.
  // ROS_INFO_NAMED("tutorial", "Detach the object from the robot");
  // move_group.detachObject(object_to_attach.id);

  // // Show text in RViz of status
  // visual_tools.deleteAllMarkers();
  // visual_tools.publishText(text_pose, "Object detached from robot", rvt::WHITE, rvt::XLARGE);
  // visual_tools.trigger();

  // /* Wait for MoveGroup to receive and process the attached collision object message */
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the new object is detached from the robot");

  // // Now, let's remove the objects from the world.
  // ROS_INFO_NAMED("tutorial", "Remove the objects from the world");
  // std::vector<std::string> object_ids;
  // object_ids.push_back(collision_object.id);
  // object_ids.push_back(object_to_attach.id);
  // planning_scene_interface.removeCollisionObjects(object_ids);

  // // Show text in RViz of status
  // visual_tools.publishText(text_pose, "Objects removed", rvt::WHITE, rvt::XLARGE);
  // visual_tools.trigger();

  // /* Wait for MoveGroup to receive and process the attached collision object message */
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object disapears");

  // // END_TUTORIAL
  // //TODO
  ros::shutdown();
  return 0;
}
