#ifndef SCENARIO_H
#define SCENARIO_H

/* cpp header */
#include <iostream>
#include <string>
#include <vector>

/* ros header */
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <swarm_ctrl_pkg/srvMultiSetpointLocal.h>
#include <swarm_ctrl_pkg/srvMultiSetpointGlobal.h>

/* others header */
#include <vehicle.h>
#include <Font7x5.h>
#include <Font8x8.h>

class Scenario
{
private:
  /* ros variables */
  ros::NodeHandle &nh_global_;
  ros::ServiceServer multi_setpoint_local_server_;
	ros::ServiceServer multi_setpoint_global_server_;

  /* swarm info */
  SwarmVehicle &swarm_;
	int num_of_vehicle_;
  std::string formation_;
  tf2::Vector3 swarm_target_local_;

  /* scenario variables */
	double angle_;
  std::vector<tf2::Vector3> scens_;
	std::vector<uint8_t> scen_hex_;
	static std::string scen_str_;
	static int scen_num_;
	ros::Time prev_;

  void swarmServiceInit();
	void release();

  void point();
  void idle(double);
  void makeCircle(double);
  void makeEllipse(double);
  void drawStringFont7x5(double);
  void drawStringFont8x8(double);
  void hexToCoord(std::vector<std::pair<int, int>> &, const uint8_t &, const int &, const bool &);

  bool multiSetpointLocal(swarm_ctrl_pkg::srvMultiSetpointLocal::Request &req,
                          swarm_ctrl_pkg::srvMultiSetpointLocal::Response &res);
  bool multiSetpointGlobal(swarm_ctrl_pkg::srvMultiSetpointGlobal::Request &req,
                           swarm_ctrl_pkg::srvMultiSetpointGlobal::Response &res);

public:
  void formationGenerator();
  Scenario(ros::NodeHandle &, SwarmVehicle &);
  Scenario(const Scenario &);
  const Scenario &operator=(const Scenario &);
  ~Scenario();
};

#endif