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

#include <sstream>
#include <iostream>
#include <math.h>
using namespace std;
#include <stdio.h>  
#define PI 3.14159265


int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_controller");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

 //Set the planning Group
  static const std::string PLANNING_GROUP = "gripx_arm";

  // The :planning_interface:`MoveGroupInterface` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

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

  //To set a new start state
  moveit::core::RobotState start_state(*move_group.getCurrentState());

  
  //Set a different planner id:
  // move_group.setPlannerId("RRTConnectkConfigDefault");

  // Planning to a Pose goal and executing


  // Quaternion transformation
  tf2::Quaternion myQuaternion, q_rot, q_new;
  

  // Create a target pose
  geometry_msgs::Pose target_pose1;

  // Define all variables needed within the loop
  double yaw = 0.0;
   int count = 0;

  while (ros::ok())
  {
    // Set a faster
    move_group.setMaxVelocityScalingFactor(1);
    cout << "Enter: X: Y: Z:" <<endl;
    cin >> target_pose1.position.x >>target_pose1.position.y >> target_pose1.position.z;

    // Calculate the needed yaw angle needed from the coordinates
    yaw = atan2(target_pose1.position.y, target_pose1.position.x);
    //Print to check
    printf("the arc tangent is %f \n", yaw);

    //Calculate the quaternions
    myQuaternion.setRPY( -1.5707, 0, 0 );  // Create this quaternion from roll/pitch/yaw (in radians)
    q_rot.setRPY(0, yaw, 0);

    //Calculate the new orientation
    myQuaternion = myQuaternion * q_rot;

    // Normalize to 1
    myQuaternion.normalize();
    // target_pose1.orientation= tf2::toMsg(myQuaternion);

    //Transfer the values of of the created Quaternion
    target_pose1.orientation.x = myQuaternion[0];
    target_pose1.orientation.y = myQuaternion[1];
    target_pose1.orientation.z = myQuaternion[2];
    target_pose1.orientation.w = myQuaternion[3]; 


    // move_group.setOrientationTarget( myQuaternion[0], myQuaternion[1], myQuaternion[2], myQuaternion[3],
    //   move_group.getEndEffectorLink());
    // move_group.setGoalOrientationTolerance(1);
    move_group.setPoseTarget(target_pose1, move_group.getEndEffectorLink());
    // move_group.setApproximateJointValueTarget(target_pose1, move_group.getEndEffectorLink());
    printf("Moving to %i target \n", count);
    move_group.move();
    ++count;
    // start_state.setFromIK(joint_model_group, target_pose1);
    // move_group.setStartState(start_state);

     // Planning a Cartesian Path by interpolating through waypoints


    //Create a vector of poses
    std::vector<geometry_msgs::Pose> waypoints;

       //Push the current endeffector pose inte the vector
    waypoints.push_back(target_pose1);

    geometry_msgs::Pose target_pose = target_pose1;

    target_pose.position.z += 0.05;
    waypoints.push_back(target_pose);  // down

    target_pose.position.z += 0.05;
    waypoints.push_back(target_pose);  // down

    target_pose.position.z += 0.05;
    waypoints.push_back(target_pose);  // up and left

    // Set a slower execution speed
    move_group.setMaxVelocityScalingFactor(0.1);

    //Give the planner more time
    move_group.setPlanningTime(15.0);

    //allow several planning attemps
    move_group.setNumPlanningAttempts(3);

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

    //Execute the trajectory
    move_group.execute(trajectory);

  }
    
  ros::shutdown();
  return 0;
}
