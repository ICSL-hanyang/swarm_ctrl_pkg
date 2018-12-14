#ifndef ROS_MSG_EXTENSION_H
#define ROS_MSG_EXTENSION_H

#include <sensor_msgs/NavSatFix.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

const sensor_msgs::NavSatFix operator+(const sensor_msgs::NavSatFix &a, const sensor_msgs::NavSatFix &b);
sensor_msgs::NavSatFix &operator+=(sensor_msgs::NavSatFix &a, const sensor_msgs::NavSatFix &b);
void transformSender(double x, double y, double z, double roll, double pitch, double yaw, ros::Time time, const std::string &frame_id, const std::string &child_frame_id);

#endif