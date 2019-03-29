#ifndef SCENARIO_H
#define SCENARIO_H

/* cpp header */
#include <iostream>
#include <string>
#include <vector>

/* ros header */
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <swarm_ctrl_pkg/srvGoToVehicle.h>
#include <swarm_ctrl_pkg/srvMultiSetpointLocal.h>
#include <swarm_ctrl_pkg/srvMultiSetpointGlobal.h>

/* others header */
#include <vehicle.h>
#include <Font7x5.h>
#include <Font8x8.h>

class Scenario
{
private:
  ros::NodeHandle &nh_global_;
	ros::ServiceServer goto_vehicle_server_;
  ros::ServiceServer multi_setpoint_local_server_;
	ros::ServiceServer multi_setpoint_global_server_;

  SwarmVehicle &swarm_;
  std::vector<Vehicle> camila_;
  std::vector<tf2::Vector3> offset_;
  std::string formation_;
  tf2::Vector3 swarm_target_local_;

  void swarmServiceInit();
	void release();

  void point();
  void idle();
  void makeCircle();
  void makeEllipse();
  void drawStringFont7x5();
  void drawStringFont8x8();
  geometry_msgs::PoseStamped getSwarmTarget() const;
  void hexToCoord(std::vector<std::pair<int, int>> &, const uint8_t &, const int &, const bool &);

  bool gotoVehicle(swarm_ctrl_pkg::srvGoToVehicle::Request &req,
                   swarm_ctrl_pkg::srvGoToVehicle::Response &res);
  bool multiSetpointLocal(swarm_ctrl_pkg::srvMultiSetpointLocal::Request &req,
                          swarm_ctrl_pkg::srvMultiSetpointLocal::Response &res);
  bool multiSetpointGlobal(swarm_ctrl_pkg::srvMultiSetpointGlobal::Request &req,
                           swarm_ctrl_pkg::srvMultiSetpointGlobal::Response &res);

public:
  void formationGenerator();
  Scenario(ros::NodeHandle &, SwarmVehicle &);
  ~Scenario();
};

#endif