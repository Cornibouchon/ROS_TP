#pragma once
#include <vector>
#include <geometry_msgs/Pose.h>



double rad2deg(double rad);

double deg2rad(double deg);

std::vector<geometry_msgs::Pose> n_interpolate_rpy(	int steps, double roll, double pitch, double yaw,
                                                      double x_offset, double y_offset, double z_offset,
                                                      geometry_msgs::Pose start_pose);

geometry_msgs::Pose start_pose_radial(double x_coord, double y_coord, double z_coord);

geometry_msgs::Pose pose_offset_cart(double x_offset, double y_offset, double z_offset,geometry_msgs::Pose pose);
geometry_msgs::Pose pose_rotation(double roll, double pitch, double yaw, geometry_msgs::Pose pose);


