#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>   //arm용
#include <mavros_msgs/SetMode.h>       //OFFBOARD 모드 설정용
#include <geometry_msgs/PoseStamped.h> //local_position 용
#include <mavros_msgs/CommandHome.h>   //set_home
#include <sensor_msgs/NavSatFix.h>     //recieve global pos
#include "swarm_ctrl_pkg/srvMultiArming.h"
#include "swarm_ctrl_pkg/srvMultiMode.h"
#include "swarm_ctrl_pkg/msgState.h" //multi_state msg
#include "swarm_ctrl_pkg/srvMultiSetPosLocal.h"
#include "swarm_ctrl_pkg/srvMultiSetVelLocal.h"
#include "swarm_ctrl_pkg/srvMultiSetHome.h"
#include "swarm_ctrl_pkg/srvMultiLanding.h"


#define NUM_DRONE 3

ros::ServiceClient arming_client[NUM_DRONE];
ros::ServiceClient set_mode_client[NUM_DRONE];
ros::ServiceClient multi_set_pos_local_client;
ros::ServiceClient multi_set_vel_local_client;
ros::ServiceClient multi_set_home_client;


ros::ServiceClient set_home_client[NUM_DRONE];
geometry_msgs::PoseStamped l_pos[NUM_DRONE];
sensor_msgs::NavSatFix g_pos[NUM_DRONE];
swarm_ctrl_pkg::msgState multi_state;
std::string group_name = "camila";
bool b_home_landing = false;
double takeoff_alt = 2.5;

bool multiArming(swarm_ctrl_pkg::srvMultiArming::Request& req,
                 swarm_ctrl_pkg::srvMultiArming::Response& res)
{
  int arming_cnt = 0;
  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = req.arming;
  swarm_ctrl_pkg::srvMultiSetHome home;
  home.request.drone_num = 0;
  for (int i = 0; i < NUM_DRONE; i++)
  {
    if (arming_client[i].call(arm_cmd) && arm_cmd.response.success)
    {
      arming_cnt++;
    }
    else
    {
      arming_cnt = 0;
    }
  }
  if (arming_cnt == NUM_DRONE)
  {
    /*home.request.drone_num = 0;
                if (multi_set_home_client.call(home) && home.response.success)
                {
                        ROS_INFO("Home reset success");
    }*/
    res.success = true;
  }
  else
  {
    for (int i = 0; i < NUM_DRONE; i++)
    {
      if (arming_client[i].call(arm_cmd) && arm_cmd.response.success)
      {
      }
    }
    ROS_WARN("Arming fail. All drones are disarmed");
    res.success = false;
  }
  return true;
}

bool multiMode(swarm_ctrl_pkg::srvMultiMode::Request& req,
               swarm_ctrl_pkg::srvMultiMode::Response& res)
{
  int mode_cnt = 0;
  swarm_ctrl_pkg::srvMultiSetPosLocal p_msg;
  mavros_msgs::SetMode set_mode;

  p_msg.request.pos_flag = true;
  p_msg.request.x = 0;
  p_msg.request.y = 0;
  p_msg.request.z = takeoff_alt;
  multi_set_pos_local_client.call(p_msg);

  set_mode.request.custom_mode = req.mode;
  for (int i = 0; i < NUM_DRONE; i++)
  {
    if (set_mode_client[i].call(set_mode) && set_mode.response.success)
    {
      mode_cnt++;
    }
    else
    {
      mode_cnt = 0;
    }
  }
  if (mode_cnt == NUM_DRONE)
  {
    res.success = true;
  }
  else
  {
    res.success = false;
  }
  return true;
}



bool multiLanding(swarm_ctrl_pkg::srvMultiLanding::Request& req,
                  swarm_ctrl_pkg::srvMultiLanding::Response& res)
{
  swarm_ctrl_pkg::srvMultiSetPosLocal p_msg;
  swarm_ctrl_pkg::srvMultiSetVelLocal v_msg;
  if (req.where == "here" || req.where == "HERE" | req.where == "Here")
  {
    p_msg.request.pos_flag = true;
    p_msg.request.x = 0;
    p_msg.request.y = 0;
    p_msg.request.z = -10;

    v_msg.request.vel_flag = true;
    v_msg.request.vel_x = 0;
    v_msg.request.vel_y = 0;
    v_msg.request.vel_z = -0.7;
    if (multi_set_pos_local_client.call(p_msg) && p_msg.response.success)
    {

      if (multi_set_vel_local_client.call(v_msg) && v_msg.response.success)
      {
        res.success = true;
      }
      else
      {
        res.success = false;
      }

      res.success = true;
    }
    else
    {
      res.success = false;
    }
  }
  /*
  else if (req.where == "home" || req.where == "HOME" | req.where == "Home")
  {
    p_msg.request.pos_flag = true;
    p_msg.request.x = 0;
    p_msg.request.y = 0;
    p_msg.request.z = l_pos[0].pose.position.z;
    if (multi_set_pos_local_client.call(p_msg) && p_msg.response.success)
    {
      b_home_landing = true;
      res.success = true;
    }
    else
    {
      res.success = false;
    }
  }*/
  return true;
}

bool multiSetHome(swarm_ctrl_pkg::srvMultiSetHome::Request& req,
                  swarm_ctrl_pkg::srvMultiSetHome::Response& res)
{
  int cnt_home = 0;
  mavros_msgs::CommandHome home_gps;
  home_gps.request.current_gps = false;
  home_gps.request.latitude = g_pos[req.drone_num].latitude;
  home_gps.request.longitude = g_pos[req.drone_num].longitude;
  home_gps.request.altitude = g_pos[req.drone_num].altitude;
  for (int i = 0; i < NUM_DRONE; i++)
  {
    if (set_home_client[i].call(home_gps) && home_gps.response.success)
    {
      ROS_INFO("Camila%d reset home position", i);
      cnt_home++;
    }
    else
    {
      cnt_home = 0;
    }
  }
  (cnt_home == NUM_DRONE) ? res.success = true : res.success = false;
  return true;
}
void LocalPosCB(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  std::stringstream stream;
  for (int i = 0; i < NUM_DRONE; i++)
  {
    stream << i;
    if (msg->header.frame_id == group_name + stream.str())
      l_pos[i] = *msg;
    stream.str("");
  }
}
void globalPosCB(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  std::stringstream stream;
  for (int i = 0; i < NUM_DRONE; i++)
  {
    stream << i;
    if (msg->header.frame_id == group_name + stream.str())
      g_pos[i] = *msg;
    stream.str("");
  }
}

void multiStateCB(const swarm_ctrl_pkg::msgState::ConstPtr& msg)
{
  multi_state = *msg;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cmd_node");
  ros::NodeHandle nh;

  ros::Subscriber local_pos_sub[NUM_DRONE];
  ros::Subscriber global_pos_sub[NUM_DRONE];
  ros::ServiceServer multi_arming_server =
      nh.advertiseService("multi_arming", multiArming);
  ros::ServiceServer multi_mode_server =
      nh.advertiseService("multi_mode", multiMode);
  ros::ServiceServer multi_landing_server =
  nh.advertiseService("multi_landing", multiLanding);
  ros::ServiceServer multi_set_home_server =
      nh.advertiseService("multi_set_home", multiSetHome);
  ros::Subscriber multi_state_sub =
      nh.subscribe("multi_state", 50, multiStateCB);
  multi_set_home_client =
      nh.serviceClient<swarm_ctrl_pkg::srvMultiSetHome>("multi_set_home");
  multi_set_pos_local_client =
      nh.serviceClient<swarm_ctrl_pkg::srvMultiSetPosLocal>(
          "multi_set_pos_local");
  multi_set_vel_local_client =
      nh.serviceClient<swarm_ctrl_pkg::srvMultiSetVelLocal>(
          "multi_set_vel_local");

  std::stringstream stream;
  std::string d_mavros_arm = "/mavros/cmd/arming";
  std::string d_mavros_mode = "/mavros/set_mode";
  std::string d_mavros_home = "/mavros/cmd/set_home";
  std::string d_mavros_l_pos = "/mavros/local_position/pose";
  std::string d_mavros_g_pos = "/mavros/global_position/global";

  for (int i = 0; i < NUM_DRONE; i++)
  {
    stream << i;
    arming_client[i] = nh.serviceClient<mavros_msgs::CommandBool>(
        group_name + stream.str() + d_mavros_arm);
    set_mode_client[i] = nh.serviceClient<mavros_msgs::SetMode>(
        group_name + stream.str() + d_mavros_mode);
    local_pos_sub[i] = nh.subscribe(group_name + stream.str() + d_mavros_l_pos,
                                    10, LocalPosCB);
    global_pos_sub[i] = nh.subscribe(group_name + stream.str() + d_mavros_g_pos,
                                     10, globalPosCB);
    set_home_client[i] = nh.serviceClient<mavros_msgs::CommandHome>(
        group_name + stream.str() + d_mavros_home);
    stream.str("");
  }

  ros::Rate rate(10.0); // period 0.01 s
  ROS_INFO("Command node started");
  while (ros::ok()){



    ros::spinOnce();
    rate.sleep();
  }


  /*
  nh.setParam("cmd_node/takeoff_altitude", 2.0);
  ros::Time set_timer;
  while (ros::ok())
  {
        nh.getParam("cmd_node/takeoff_altitude", takeoff_al);
    if(b_home_landing && (l_pos[0].pose.position.x < 0.5 ||
    l_pos[0].pose.position.x > -0.5)){
            set_timer = ros::Time::now() + ros::Duration(3.0);
            b_home_landing = false;
    }
    if( (set_timer - ros::Time::now()) < ros::Duration(0.2)){
            swarm_ctrl_pkg::srvMultiSetPosLocal p_msg;
            swarm_ctrl_pkg::srvMultiSetVelLocal v_msg;
            p_msg.request.pos_flag = false;
            v_msg.request.vel_flag = true;
            v_msg.request.vel_z = -0.7;
            multi_set_pos_local_client.call(p_msg);
            multi_set_vel_local_client.call(v_msg);
    }
        ros::spinOnce();
        rate.sleep();
  }*/

  ros::spin();
  return 0;
}
