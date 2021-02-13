#include <controllers/utils.h>
#include <geometry_msgs/Pose.h>
// Used for the convertion from euler to quaternion see: http://wiki.ros.org/tf2/Tutorials/Quaternions
#include <tf2/LinearMath/Quaternion.h>
// Used to  transform between tf2_quaternion and geometry_msgs quaternion
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define PI 3.14159265


double rad2deg(double rad)
{
  return rad*180/PI;
}

double deg2rad(double deg)
{
  return deg*PI/180;
}

//Takes values for roll pitch and yaw in degrees, and returs a vector list which can be used for planning
std::vector<geometry_msgs::Pose> n_interpolate_rpy(	int steps, 
													double roll,
													double pitch,
													double yaw,
													double z_offset,
													geometry_msgs::Pose start_pose)
{	
	std::vector<geometry_msgs::Pose> waypoints;
	// Push the current endeffector pose inte the vector
    waypoints.push_back(start_pose);
    // Create needed quaternions for transformation
    tf2::Quaternion q_orig, q_rot, q_new;
    //Create rotation quaternion
    q_rot.setRPY(deg2rad(roll/(steps+1)), deg2rad(pitch/(steps+1)), deg2rad(yaw/(steps+1)));

    for(int i=0; i<=steps; i++){
    	// Get the original orientation of 'targer_pose'; store in q_orig
    	tf2::convert(start_pose.orientation , q_orig);
    	q_new = q_rot*q_orig;
	    q_new.normalize();
	    //Put the new orientation back into the target pose
	    tf2::convert(q_new, start_pose.orientation);
	    start_pose.position.z += z_offset/steps;
	    waypoints.push_back(start_pose);
    }

    return waypoints;
}
