#include <ros/ros.h>
#include <mavros_msgs/State.h>       //state check 메세지 활용용
#include "swarm_ctrl_pkg/msgState.h" //multi_state msg

#define NUM_DRONE 0

typedef void (*FuncPtr)(const mavros_msgs::State::ConstPtr&);
mavros_msgs::State current_state[5];
std::string mode[5];

void stateCB0(const mavros_msgs::State::ConstPtr& msg)
{
  current_state[0] = *msg;
  mode[0] = current_state[0].mode;
}
void stateCB1(const mavros_msgs::State::ConstPtr& msg)
{
  current_state[1] = *msg;
  mode[1] = current_state[1].mode;
}
void stateCB2(const mavros_msgs::State::ConstPtr& msg)
{
  current_state[2] = *msg;
  mode[2] = current_state[2].mode;
}

void stateCB3(const mavros_msgs::State::ConstPtr& msg){
  current_state[3] = *msg;
  mode[3]=current_state[3].mode;
}


void stateCB4(const mavros_msgs::State::ConstPtr& msg){
  current_state[4] = *msg;
  mode[4]=current_state[4].mode;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "state_node");

  ros::NodeHandle nh;
  ros::Subscriber state_sub[5];
  ros::Publisher multi_state_pub =
  nh.advertise<swarm_ctrl_pkg::msgState>("multi_state", 50);

  std::stringstream stream;
  std::string group_name = "camila";
  std::string d_mavros_state = "/mavros/state";

  FuncPtr stateFP[5] = {stateCB0, stateCB1,
    stateCB2, stateCB3, stateCB4};
    mavros_msgs::State pre_state[5];

    stream << NUM_DRONE;
    state_sub[NUM_DRONE] = nh.subscribe(group_name + stream.str() + d_mavros_state, 10,
      stateFP[NUM_DRONE]);
    //std::cout << group_name + stream.str() + d_mavros_state<<std::endl;
    stream.str("");
  ros::Rate rate(20.0); // period 0.05

  pre_state[NUM_DRONE].connected = true;
  pre_state[NUM_DRONE].mode = "OFFBOARD";
  pre_state[NUM_DRONE].armed = true;


  ROS_INFO("State check start");
  while (ros::ok())
  {
    swarm_ctrl_pkg::msgState msg;
    
    (current_state[NUM_DRONE].connected == true) ? msg.connected = true : msg.connected = false;
    if (current_state[NUM_DRONE].connected != pre_state[NUM_DRONE].connected)
    {
      if (current_state[NUM_DRONE].connected == true)
      {
        ROS_WARN("Camila%d is connected", NUM_DRONE);
      }
      else
      {
        ROS_WARN("Camila%d is not connected", NUM_DRONE);
      }
    }
    (current_state[NUM_DRONE].mode == "OFFBOARD") ? msg.mode = true : msg.mode = false;
    if (current_state[NUM_DRONE].mode != pre_state[NUM_DRONE].mode)
    {
      ROS_INFO("Camila%d is %s mode", NUM_DRONE, mode[NUM_DRONE].c_str());
    }
    (current_state[NUM_DRONE].armed == true) ? msg.armed = true : msg.armed = false;
    if (current_state[NUM_DRONE].armed != pre_state[NUM_DRONE].armed)
    {
      if (current_state[NUM_DRONE].armed == 128)
      {
        ROS_INFO("Camila%d is armed", NUM_DRONE);
      }
      else
      {

        ROS_INFO("Camila%d is disarmed", NUM_DRONE);
      }
    }
    pre_state[NUM_DRONE] = current_state[NUM_DRONE];

    multi_state_pub.publish(msg);

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
