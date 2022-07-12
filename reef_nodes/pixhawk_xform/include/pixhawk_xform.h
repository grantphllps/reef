//
// pixhawk_xform.h
//
// This node flips the axes on the mocap system
// so the correct positional data can be streamed
// to the Pixhawk via mavros
//
// Created by Adam Plowcha
//

#ifndef PIXHAWK_XFORM_H_
#define PIXHAWK_XFORM_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <cmath>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Range.h>

//Publishers and their var
ros::Publisher xform_pub;


//changed to match vicon vs. mocap
geometry_msgs::TransformStamped vicon;

geometry_msgs::PoseStamped xformed;

//Subscribers

ros::Subscriber vicon_sub;
void callbackVicon(const geometry_msgs::TransformStampedConstPtr &msg);

//Helper methods - these are generally usefully
geometry_msgs::Pose getDegFromQuat(geometry_msgs::Pose p);
double rad_to_deg(double x) {return x * (180.0 / 3.14159);}
double deg_to_rad(double x) {return (x * 3.14159) / 180.0;}

#endif // PIXHAWK_XFORM_H_
