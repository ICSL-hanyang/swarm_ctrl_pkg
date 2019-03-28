#ifndef SCENARIO_H
#define SCENARIO_H

/* cpp header */
#include <iostream>
#include <string>
#include <vector>

/* ros header */
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

/* others header */
#include <vehicle.h>
#include <Font7x5.h>
#include <Font8x8.h>

class Scenario
{
  private:
    ros::NodeHandle &nh_global_;
    SwarmVehicle &swarm_;
    const std::vector<Vehicle> *camila_;
    const std::vector<tf2::Vector3> *offset_;
    const std::string *formation_;
    const tf2::Vector3 *swarm_target_local_;
    void point();
    void idle();
    void makeCircle();
    void makeEllipse();
    void drawStringFont7x5();
    void drawStringFont8x8();
    geometry_msgs::PoseStamped getSwarmTarget() const; 
    void hexToCoord(std::vector<std::pair<int, int>> &, const uint8_t &, const int &, const bool &);

  public:
    void formationGenerator();
    Scenario(ros::NodeHandle &, SwarmVehicle &);
};

#endif