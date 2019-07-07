#ifndef MISSION_H
#define MISSION_H
#include <iostream>
#include <vector>
#include <cmath>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
<<<<<<< HEAD
#include <swarm_ctrl_pkg/srvMultiSetpointLocal.h>
=======
#include <std_msgs/Empty.h>
#include <vehicle.h>
>>>>>>> 2d28d49... trigger

class Mission
{
private:
    ros::NodeHandle nh_;
	ros::Subscriber local_pos_sub_;
    ros::ServiceClient setpoint_client_;
    std::vector<tf2::Vector3> waypoints_;
    int wp_index_;
    double initial_yaw_;
    tf2::Vector3 cur_waypoint_;
	geometry_msgs::PoseStamped cur_local_;
	void localPositionCB(const geometry_msgs::PoseStamped::ConstPtr &);
    
public:
    Mission();
    ~Mission();
    void clear();
    void pushWaypoint(const tf2::Vector3 &);
    bool checkReached();
    void findYaw();
    void run();
};

#endif