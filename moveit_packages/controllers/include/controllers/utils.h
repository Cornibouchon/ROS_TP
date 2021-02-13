#pragma once
#include <vector>
#include <geometry_msgs/Pose.h>



double rad2deg(double rad);

double deg2rad(double deg);

std::vector<geometry_msgs::Pose> n_interpolate_rpy(	int steps, 
													double roll, 
													double pitch, 
													double yaw,
													double y_offset,
													geometry_msgs::Pose msg);



