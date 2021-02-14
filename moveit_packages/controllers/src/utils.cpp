#include <controllers/utils.h>
#include <geometry_msgs/Pose.h>
// Used for the convertion from euler to quaternion see: http://wiki.ros.org/tf2/Tutorials/Quaternions
#include <tf2/LinearMath/Quaternion.h>
// Used to  transform between tf2_quaternion and geometry_msgs quaternion
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <math.h>

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
std::vector<geometry_msgs::Pose> n_interpolate_rpy(	int steps, double roll, double pitch, double yaw,
                                                    double x_offset, double y_offset, double z_offset,
													geometry_msgs::Pose start_pose)
{	
	std::vector<geometry_msgs::Pose> waypoints;
	// Push the current EEF pose into the vector
    waypoints.push_back(start_pose);

    for(int i=0; i<=steps; i++){
        start_pose = pose_rotation(roll/(steps+1), pitch/(steps+1), yaw/(steps+1), start_pose);
        start_pose = pose_offset_cart(-x_offset/(steps+1),-y_offset/(steps+1), -qz_offset/steps,start_pose);
	    start_pose.position.z += z_offset/steps;
	    waypoints.push_back(start_pose);
    }
    return waypoints;
}

geometry_msgs::Pose start_pose_radial(double x_coord, double y_coord, double z_coord)
{
    double yaw =0.0;
    tf2::Quaternion myQuaternion;
    geometry_msgs::Pose target_pose;
    // Calculate the needed yaw angle needed from the coordinates
    yaw = atan2(y_coord, x_coord);
    //Calculate the quaternions from roll, pitch and yaw
    myQuaternion.setRPY( -PI/2, 0, PI + yaw );
    // Normalize to 1
    myQuaternion.normalize();
    //Transfer the values of of the created Quaternion
    target_pose.orientation.x = myQuaternion[0];
    target_pose.orientation.y = myQuaternion[1];
    target_pose.orientation.z = myQuaternion[2];
    target_pose.orientation.w = myQuaternion[3];
    target_pose.position.x    =x_coord;
    target_pose.position.y    =y_coord;
    target_pose.position.z    =z_coord;

    return target_pose;
}

geometry_msgs::Pose pose_offset_cart(double x_offset, double y_offset, double z_offset,geometry_msgs::Pose pose)
{
    pose.position.x -= x_offset;
    pose.position.y -= y_offset;
    pose.position.z -= z_offset;

    return pose;
}

geometry_msgs::Pose pose_rotation(double roll, double pitch, double yaw, geometry_msgs::Pose pose)
{
    tf2::Quaternion q_orig, q_rot, q_new;
    //Create rotation quaternion
    q_rot.setRPY(deg2rad(roll), deg2rad(pitch), deg2rad(yaw));
    //Extract the original quaternion, apply rotation and normalize
    tf2::convert(pose.orientation , q_orig);
    q_new = q_rot*q_orig;
    q_new.normalize();
    //Put the new orientation back into the target pose
    tf2::convert(q_new, pose.orientation);

    return pose;
}