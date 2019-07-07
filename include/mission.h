#ifndef MISSION_H
#define MISSION_H
#include <iostream>
#include <vector>
#include <cmath>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/Empty.h>
#include <vehicle.h>

class Mission
{
private:
	ros::Subscriber local_pos_sub_;
    std::vector<tf2::Vector3> waypoints_;
    tf2::Vector3 cur_waypoint_;
	geometry_msgs::PoseStamped cur_local_;
	void localPositionCB(const geometry_msgs::PoseStamped::ConstPtr &);
    
public:
    Mission();
    ~Mission();
    void clear();
    void pushWaypoint(tf2::Vector3 &);
    bool checkReached() const;
};

#endif