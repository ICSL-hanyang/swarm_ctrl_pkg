#ifndef VEHICLE_H
#define VEHICLE_H

#include <ros/ros.h>
#include <stdio.h>
#include <string>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>  
#include <sensor_msgs/BatteryState.h>
#include <swarm_ctrl_pkg/srvMultiArming.h>
#include <swarm_ctrl_pkg/srvMultiMode.h>
#include <swarm_ctrl_pkg/srvMultiSetHome.h>
#include <swarm_ctrl_pkg/srvMultiSetpointLocal.h>
#include <swarm_ctrl_pkg/srvMultiSetpointGlobal.h>

typedef struct str_info{
	int system_id;
	std::string vehicle_name;
}VehicleInfo;

/* 	Drone modes : 
	MANUAL, STABILIZED,	OFFBOARD, ACRO, ALTCTL,	POSCTL,	RATTITUDE, 
	AUTO.TAKEOFF, AUTO.LAND, AUTO.MISSION, AUTO.LOITER,	AUTO.RTL,
	AUTO.RTGS, AUTO.READY,*/

class Vehicle{
	private:
		ros::NodeHandle nh;

		/* ros subscriber*/
		ros::Subscriber state_sub;
		ros::Subscriber battery_sub;
		ros::Subscriber local_pos_sub;
		ros::Subscriber global_pos_sub;

		/* ros publisher*/
		ros::Publisher setpoint_local_pub;
		ros::Publisher setpoint_global_pub;

		/* ros service client*/
		ros::ServiceClient arming_client;
		ros::ServiceClient set_mode_client;
		ros::ServiceClient set_home_client;

		/*drone state*/
		VehicleInfo vehicle_info;
		mavros_msgs::State cur_state;
		sensor_msgs::BatteryState cur_battery;
		
		/* global coordinate*/
		sensor_msgs::NavSatFix home_global;
		sensor_msgs::NavSatFix cur_global;
		sensor_msgs::NavSatFix tar_global;
		
		/* local coordinate*/
		geometry_msgs::PoseStamped home_local;
		geometry_msgs::PoseStamped cur_local;
		geometry_msgs::PoseStamped tar_local;

		bool setpoint_publish_flag;

		/*fermware version=> diagnositic_msgs/DiagnosticStatus*/

		void vehicleInit();

	public:
		Vehicle();
		Vehicle(VehicleInfo _vehicle_info);

		/*main drone function*/
		bool arming(bool _arm_state);
		bool setMode(std::string _mode);
		void gotoGlobal(sensor_msgs::NavSatFix _tar_global);

		void gotoLocal(geometry_msgs::PoseStamped _tar_local);
		
		void setVehicleInfo(VehicleInfo new_vehicle_info);
		VehicleInfo getInfo();
		void stateCB(const mavros_msgs::State::ConstPtr& msg);
		mavros_msgs::State getState();
		void batteryCB(const sensor_msgs::BatteryState::ConstPtr& msg);
		sensor_msgs::BatteryState getBattery();

		//global position
		bool setHomeGlobal();
		sensor_msgs::NavSatFix getHomeGlobal();
		void globalPositionCB(const sensor_msgs::NavSatFix::ConstPtr& msg);
		sensor_msgs::NavSatFix getGlobalPosition();
		sensor_msgs::NavSatFix getTargetGlobal();

		//local position
		void setHomeLocal();
		geometry_msgs::PoseStamped getHomeLocal();
		void localPositionCB(const geometry_msgs::PoseStamped::ConstPtr& msg);
		geometry_msgs::PoseStamped getLocalPosition();
		geometry_msgs::PoseStamped getTargetLocal();

		bool isPublish();
};

class SwarmVehicle{
	private:
		ros::NodeHandle nh;

		ros::ServiceServer multi_arming_server;
		ros::ServiceServer multi_mode_server;
		ros::ServiceServer multi_sethome_server;
		ros::ServiceServer multi_setpoint_local_server;
		ros::ServiceServer multi_setpoint_global_server;

		//swarm_info
		Vehicle *camila;
		std::string swarm_name;
		int num_of_vehicle;
		
		std::string formation;
		double min_length;
		bool multi_setpoint_publish_flag;

		geometry_msgs::PoseStamped swarm_position_local;
		sensor_msgs::NavSatFix swarm_position_global;
	public:
		SwarmVehicle(std::string _swarm_name = "camila", int _num_of_vehicle = 1); //have to add default value

		void setSwarmInfo(std::string _swarm_name, int _num_of_vehicle);
		std::string getSwarmInfo();

		bool multiArming(mavros_msgs::CommandBool::Request& req,
			mavros_msgs::CommandBool::Response& res);
		bool multiMode(swarm_ctrl_pkg::srvMultiMode::Request& req,
			swarm_ctrl_pkg::srvMultiMode::Response& res);
		bool multiSetHome(swarm_ctrl_pkg::srvMultiSetHome::Request& req,
			swarm_ctrl_pkg::srvMultiSetHome::Response& res);
		bool multiSetpointLocal(swarm_ctrl_pkg::srvMultiSetpointLocal::Request& req,
			swarm_ctrl_pkg::srvMultiSetpointLocal::Response& res);
		bool multiSetpointGlobal(swarm_ctrl_pkg::srvMultiSetpointGlobal::Request& req,
			swarm_ctrl_pkg::srvMultiSetpointGlobal::Response& res);

		void gotoGlobal(sensor_msgs::NavSatFix _tar_global);
		void gotoLocal(geometry_msgs::PoseStamped _tar_local);

		bool isPublish();
};

#endif