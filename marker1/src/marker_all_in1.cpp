// Smae as Marker, but all code within one file

#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

//Include the custom message point_msg.cpp
#include <marker1/point_msg.h>

using namespace std;
//function prototype to create marker
void create_marker(float x_cart, float y_cart, float z_cart, int id, int action);

//TODO: Pass the message arguments to the real variables
void chatterCallback(const marker1::point_msg::ConstPtr& msg)
{
  create_marker(msg->x_coord, msg->y_coord, msg->z_coord, msg->id, msg->action);
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv) 
{
  //string namespace1 = "create_marker1";
  //string namespace2 = "create_marker2";

  // Initialize ros and create the node "nambespace"
	ros::init(argc, argv, "namespace");
  //ros::init(argc, argv, namespace2);
  ros::NodeHandle n;
  // Subscribe to the publisher
  ros::Subscriber sub = n.subscribe("chatter1", 1000, chatterCallback);

  ros::spin();
  /* Use the function to create some markers
    for (int i = 0; i < 10; ++i)
    {
      create_marker(i,i,i,i);
    }

      /*while (ros::ok()) {

    /* Publish the marker
    while (marker_pub.getNumSubscribers() < 1) {
      if (!ros::ok()) {
        return 0;
      }
      
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
      */
    //}
  }

void create_marker(float x_cart, float y_cart, float z_cart, int id, int action)
{
  //ros::init(argc, argv, "create_marker");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("/visualization_marker", 1);

  // link shape to Strawberry
  // uint32_t shape = visualization_msgs::Marker::MESH_RESOURCE;
  uint32_t shape = visualization_msgs::Marker::SPHERE;

  for(int i =0; i<5;++i)
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/base_link";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "namespace";
    marker.id = id;

    // Set the marker type.
    marker.type = shape;

    //Linkk to the strawberry
    // marker.mesh_resource = "package://marker1/marker1_description/meshes/strawknight.stl";


    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    if(action == 1){
      marker.action = visualization_msgs::Marker::ADD;
    }
    else{
      marker.action = visualization_msgs::Marker::DELETE;
    }
    
    // marker.action = visualization_msgs::Marker::ADD;
    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = x_cart;
    marker.pose.position.y = y_cart;
    marker.pose.position.z = z_cart;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.045;
    marker.scale.y = 0.045;
    marker.scale.z = 0.045;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    //Display the STL
    //marker.mesh_resource = "package://marker1_description/meshes/Strawberry_v1.obj";

    marker.lifetime = ros::Duration();

    /* Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    } */
    marker_pub.publish(marker);
    r.sleep();
  }
}