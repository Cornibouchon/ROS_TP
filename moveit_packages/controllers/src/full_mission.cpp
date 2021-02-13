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

//Used to import the custom message from marker1
#include "std_msgs/String.h"
//Include the custom message point_msg.cpp
#include <marker1/point_msg.h>


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

  // We will use the :planning_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Getting Basic Information
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

  //define the target goal to place the strawberry
  q_basket.setRPY( -1.5707, 0, PI + atan2(0, 0.8));  // Create this quaternion from roll/pitch/yaw (in radians)
  // Normalize to 1
  q_basket.normalize();
  //Transfer the values of of the created Quaternion
  basket_pose.orientation.x = q_basket[0];
  basket_pose.orientation.y = q_basket[1];
  basket_pose.orientation.z = q_basket[2];
  basket_pose.orientation.w = q_basket[3];
  basket_pose.position.x    = 0.8;
  basket_pose.position.y    = 0.0;
  basket_pose.position.z    = 0.3;



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

    //Define a collsion box to ensure no other strawberries are harmed during retrieval
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group.getPlanningFrame();
    collision_object.id = "box1";
    //Define the box
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    double box_size = 0.1;
    primitive.dimensions[0] = box_size;
    primitive.dimensions[1] = box_size;
    primitive.dimensions[2] = box_size;
    //Define the box pose relative to frame id
    geometry_msgs::Pose box_pose;
    box_pose.orientation.x = myQuaternion[0];
    box_pose.orientation.x = myQuaternion[1];
    box_pose.orientation.x = myQuaternion[2];
    box_pose.orientation.x = myQuaternion[3];

    box_pose.position.x = target_pose1.position.x + 0.85*box_size;
    box_pose.position.y = target_pose1.position.y + 0.85*box_size;
    box_pose.position.z = target_pose1.position.z + 0.85*box_size; 

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    //Display the box
    // ROS_INFO_NAMED("tutorial", "Add an object into the world");
    // planning_scene_interface.addCollisionObjects(collision_objects);


    //Define and substract the offset for the cartesian path planning
    double offset = 0.10;
    target_pose1.position.z -= offset;

    // move_group.setOrientationTarget( myQuaternion[0], myQuaternion[1], myQuaternion[2], myQuaternion[3],
    //   move_group.getEndEffectorLink());
    // move_group.setGoalOrientationTolerance(1);
    move_group.setPoseTarget(target_pose1, move_group.getEndEffectorLink());

    // Used to debug
    // printf("The getPoseReferenceFrame ist\n");
    // cout << move_group.getPoseReferenceFrame();
    // printf("The EndeffectorLink is :\n");
    // cout << move_group.getEndEffectorLink();
    // move_group.setApproximateJointValueTarget(target_pose1, move_group.getEndEffectorLink());

    //Set a orientation tolerance
    move_group.setGoalOrientationTolerance(0.02);
    printf("Moving to %i target \n", count);
    move_group.move();
    ++count;
    // start_state.setFromIK(joint_model_group, target_pose1);
    // move_group.setStartState(start_state);

     // Planning a Cartesian Path by interpolating through waypoints

    //Create a vector of poses
    std::vector<geometry_msgs::Pose> waypoints;

       // Push the current endeffector pose inte the vector
    waypoints.push_back(target_pose1);
    geometry_msgs::Pose target_pose = target_pose1;
    //Push target poses
    target_pose.position.z += offset/3;
    waypoints.push_back(target_pose);  // down

    target_pose.position.z += offset/3;
    waypoints.push_back(target_pose);  // down

    target_pose.position.z += offset/3;
    waypoints.push_back(target_pose);  // up

    target_pose.position.z += offset/3;
    waypoints.push_back(target_pose);  // up


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
    const double eef_step = 0.005;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("tutorial", "Visualizing plan %i (Cartesian path) (%.2f%% acheived)",count,  fraction * 100.0);

    //Execute the trajectory
    move_group.execute(trajectory);

    // Change the message to destroy the marker and publish it
    msg.action = 2;
    publisher.publish(msg);

    //attach the collision object
    // move_group.attachObject(collision_object.id);

    //Place the picked strawberry in the basket
    move_group.setMaxVelocityScalingFactor(2);
    move_group.setPoseTarget(basket_pose, move_group.getEndEffectorLink());
    printf("Moving to basket");
    move_group.move();

  }
    
  ros::shutdown();
  return 0;
}
