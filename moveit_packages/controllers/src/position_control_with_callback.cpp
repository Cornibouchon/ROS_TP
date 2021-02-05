#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

// includes for subscribing
#include "ros/ros.h"
#include "std_msgs/String.h"
//Include the custom message point_msg.cpp
#include <marker1/point_msg.h>

#include <sstream>
using namespace std;

void send_pose(float x_cart, float y_cart, float z_cart);

void chatterCallback(const marker1::point_msg::ConstPtr& msg)
{
  send_pose(msg->x_coord, msg->y_coord, msg->z_coord);
}
  

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_controller_with_callback");
  ros::NodeHandle n;
  // ros::AsyncSpinner spinner(1);
  // spinner.start();

 //Set the planning Group
  static const std::string PLANNING_GROUP = "gripx_arm";

  // The :planning_interface:`MoveGroupInterface` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  ros::Subscriber sub = n.subscribe("chatter1", 1000, chatterCallback);

  
  ros::spin();
}

void send_pose(float x_cart, float y_cart, float z_cart)
{
  //Set the planning Group
  static const std::string PLANNING_GROUP = "gripx_arm";

  // The :planning_interface:`MoveGroupInterface` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  // Planning to a Position goal and executing
  // ^^^^^^^^^^^^^^^^^^^^^^^
  move_group.setPositionTarget( x_cart, y_cart ,z_cart , move_group.getEndEffectorLink());
  move_group.move();

}