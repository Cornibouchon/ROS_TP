// position_publisher.cpp
#include "ros/ros.h"
#include "std_msgs/String.h"
//Include the custom message point_msg.cpp
#include <marker1/point_msg.h>

#include <visualization_msgs/Marker.h>
//Used for input output
#include <sstream>
#include <iostream>
using namespace std;

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "position_publisher");
  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<marker1::point_msg>("chatter1", 1000);

  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;

  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    marker1::point_msg msg;

    //std::stringstream ss;

    // Ask for the coordinates + id from the user
    cout << "Enter: X; Y: Z; ID; Action" <<endl;
    // Transfer dthe ID 
    cin >> msg.x_coord >> msg.y_coord >>msg.z_coord >>msg.id >>msg.action;


    //msg.data = ss.str();

    //ROS_INFO("%s", msg.x_coord.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}