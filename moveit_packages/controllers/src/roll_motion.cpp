#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>



#include <sstream>
#include <iostream>
#include <math.h>
using namespace std;
#include <stdio.h>  
#define PI 3.14159265

//Used to import the custom message from marker1
#include "std_msgs/String.h"
//Include the custom message point_msg.cpp
#include <marker1/point_msg.h>

#include <controllers/utils.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "roll_motion");
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
  //To set a new start state
  moveit::core::RobotState start_state(*move_group.getCurrentState());

  //Publisch to the topic "chatter1" to create or destroy markers
  ros::Publisher publisher = node_handle.advertise<marker1::point_msg>("chatter1", 1000);
  
  //Set a different planner id:
  // move_group.setPlannerId("RRTConnectkConfigDefault");

  // Planning to a Pose goal and executing


  // Quaternion transformation
  tf2::Quaternion myQuaternion, q_basket;
  

  // Create a target pose
  geometry_msgs::Pose target_pose1, basket_pose;

  // Define all variables needed within the loop
  double yaw = 0.0;
   int count = 0;



  while (ros::ok())
  {
    //initialize message object to be published
    marker1::point_msg msg;
    // Set a faster speed and several planning attempts
    move_group.setMaxVelocityScalingFactor(1);
    move_group.setNumPlanningAttempts(3);

    //Read coordinates from terminal and store in target pose
    cout << "Enter: X: Y: Z:" <<endl;
    cin >> target_pose1.position.x >>target_pose1.position.y >> target_pose1.position.z;

    // Calculate the needed yaw angle needed from the coordinates
    yaw = atan2(target_pose1.position.y, target_pose1.position.x);
    //Calculate the quaternions
    myQuaternion.setRPY( -1.5707, 0, PI + yaw );  // Create this quaternion from roll/pitch/yaw (in radians)
    // Normalize to 1
    myQuaternion.normalize();

    //Transfer the values of of the created Quaternion
    target_pose1.orientation.x = myQuaternion[0];
    target_pose1.orientation.y = myQuaternion[1];
    target_pose1.orientation.z = myQuaternion[2];
    target_pose1.orientation.w = myQuaternion[3];


    //fill the marker_message
    msg.x_coord = target_pose1.position.x;
    msg.y_coord = target_pose1.position.y;
    msg.z_coord = target_pose1.position.z;
    msg.id      = count;
    msg.action  = 1;
    //advertise the msg
    publisher.publish(msg);


    //Define and substract the offset for the cartesian path planning
    double offset = 0.10;
    target_pose1.position.z -= offset;

    //Set Target Pose
    move_group.setPoseTarget(target_pose1, move_group.getEndEffectorLink());
    //Set a orientation tolerance
    move_group.setGoalOrientationTolerance(0.02);
    printf("Moving to %i target \n", count);
    move_group.move();
    ++count;

    // implement simple roll motion by planning n waypoints and interpolating the roll from start poser
    //Create a vector of poses
    std::vector<geometry_msgs::Pose> waypoints1;
    waypoints1 = n_interpolate_rpy(4, 30, 0, 0, 0.1, target_pose1 );



    // Set a slower execution speed
    move_group.setMaxVelocityScalingFactor(0.1);

    //Give the planner more time
    move_group.setPlanningTime(15.0);

    // // We want the Cartesian path to be interpolated at a resolution of 1 cm
    // // which is why we will specify 0.01 as the max step in Cartesian
    // // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
    // // Warning - disabling the jump threshold while operating real hardware can cause
    // // large unpredictable motions of redundant joints and could be a safety issue
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.005;
    double fraction = move_group.computeCartesianPath(waypoints1, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("tutorial", "Visualizing plan %i (Cartesian path) (%.2f%% acheived)",count,  fraction * 100.0);

    //Execute the trajectory
    move_group.execute(trajectory);

    // Change the message to destroy the marker and publish it
    msg.action = 2;
    publisher.publish(msg);

    //attach the collision object
    // move_group.attachObject(collision_object.id);
  }
    
  ros::shutdown();
  return 0;
}


