#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h> //set position 용
#include <mavros_msgs/CommandBool.h> //arm용
#include <mavros_msgs/SetMode.h> //OFFBOARD 모드 설정용
#include <mavros_msgs/State.h> //mavros 메세지 활용용
#include <mavros_msgs/CommandHome.h> //set_home 
#include <sensor_msgs/NavSatFix.h> //set_home 
#include <sensor_msgs/TimeReference.h> //tf_old data
#define NUM_DRONE 2

//Parameter
int mode = 0;
int pre_mode = 0;

double coor[3] = {0.0,};
double com_x = 0.0;
double com_y = 0.0;
double com_z = 2;

double offset = 2.0;

void set_positon(double x, double y, double z);

mavros_msgs::State current_state[NUM_DRONE];
sensor_msgs::TimeReference current_time[NUM_DRONE];
sensor_msgs::NavSatFix g_pos[NUM_DRONE];
geometry_msgs::PoseStamped l_pos[NUM_DRONE];

void state_cb1(const mavros_msgs::State::ConstPtr& msg){
    current_state[0] = *msg;
}

void state_cb2(const mavros_msgs::State::ConstPtr& msg){
    current_state[1] = *msg;
}

void global_pos_cb1(const sensor_msgs::NavSatFix::ConstPtr& msg){
    g_pos[0] = *msg;
}

void global_pos_cb2(const sensor_msgs::NavSatFix::ConstPtr& msg){
    g_pos[1] = *msg;
}

void time_ref_cb(const sensor_msgs::TimeReference::ConstPtr& msg){
	current_time[0] = *msg;
    for(int i = 0; i< NUM_DRONE ; i++){
        g_pos[i].header.stamp = current_time[0].time_ref;
        l_pos[i].header.stamp = current_time[0].time_ref;
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "control_node");

    ROS_INFO("Control node executed");

    ros::NodeHandle nh;
    ros::Subscriber state_sub[NUM_DRONE];
    ros::Publisher local_pos_pub[NUM_DRONE];
    ros::Subscriber global_pos_sub[NUM_DRONE];
    ros::Publisher global_pos_pub[NUM_DRONE];
    ros::ServiceClient arming_client[NUM_DRONE];
    ros::ServiceClient set_mode_client[NUM_DRONE];
    ros::ServiceClient set_home_client[NUM_DRONE];
    ros::Subscriber time_ref_sub = nh.subscribe<sensor_msgs::TimeReference> ("d1/mavros/time_reference", 1, time_ref_cb);
    
    state_sub[0] = nh.subscribe<mavros_msgs::State> ("d1/mavros/state", 10, state_cb1);
    state_sub[1] = nh.subscribe<mavros_msgs::State> ("d2/mavros/state", 10, state_cb2);
    local_pos_pub[0] = nh.advertise<geometry_msgs::PoseStamped> ("d1/mavros/setpoint_position/local", 10);
    local_pos_pub[1] = nh.advertise<geometry_msgs::PoseStamped> ("d2/mavros/setpoint_position/local", 10);
    global_pos_sub[0] = nh.subscribe<sensor_msgs::NavSatFix> ("d1/mavros/global_position/global", 10, global_pos_cb1);
    global_pos_sub[1] = nh.subscribe<sensor_msgs::NavSatFix> ("d2/mavros/global_position/global", 10, global_pos_cb2);
    global_pos_pub[0] = nh.advertise<sensor_msgs::NavSatFix> ("d1/mavros/global_position/global", 10);
    global_pos_pub[1] = nh.advertise<sensor_msgs::NavSatFix> ("d2/mavros/global_position/global", 10);
    arming_client[0] = nh.serviceClient<mavros_msgs::CommandBool> ("d1/mavros/cmd/arming");
    arming_client[1] = nh.serviceClient<mavros_msgs::CommandBool> ("d2/mavros/cmd/arming");
    set_mode_client[0] = nh.serviceClient<mavros_msgs::SetMode> ("d1/mavros/set_mode");
    set_mode_client[1] = nh.serviceClient<mavros_msgs::SetMode> ("d2/mavros/set_mode");
    set_home_client[0] = nh.serviceClient<mavros_msgs::CommandHome> ("d1/mavros/set_home");
    set_home_client[1] = nh.serviceClient<mavros_msgs::CommandHome> ("d2/mavros/set_home");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(10.0); //period 0.01 s

    // wait for FCU connection
    while(ros::ok() && current_state[0].connected && current_state[1].connected){
        ROS_INFO("All Drone connected");
		ros::spinOnce();
        rate.sleep();
    }

    //reset home position
    mavros_msgs::CommandHome set_home;
    set_home.request.current_gps = true;
    while(ros::ok() && set_home_client[0].call(set_home) && set_home.response.success){
    	ROS_INFO("Camila1 reset home positon");
        set_home.request.current_gps = false;
        set_home.request.latitude = g_pos[0].latitude;
        set_home.request.longitude = g_pos[0].longitude;
        set_home.request.altitude = g_pos[0].altitude;
        for(int i =1 ; i<NUM_DRONE ; i++){
            if(ros::ok() && set_home_client[i].call(set_home)  && set_home.response.success){
                ROS_INFO("Camila%d reset home position", i+1);
            }
        }
        set_home.request.current_gps = true;
    	ros::spinOnce();
        rate.sleep();
    }

    nh.setParam("control_node/x", 0.0);
    nh.setParam("control_node/y", 0.0);
    nh.setParam("control_node/z", 2.0);
    
    //initial position
    l_pos[0].pose.position.x = 0;
    l_pos[0].pose.position.y = 0;
    l_pos[0].pose.position.z = 2;

    for(int i=1 ; i<NUM_DRONE ; i++)
		l_pos[i] = l_pos[0];

	l_pos[1].pose.position.x += offset;

    //send a few setpoints before starting
    for(int i = 50 ; ros::ok() && i > 0 ; --i){
        for(int j= 0 ; j < NUM_DRONE ; j++)
    		local_pos_pub[j].publish(l_pos[j]);
        ros::spinOnce();
        rate.sleep();
    }

    //switch mode to OFFBOARD
    mavros_msgs::SetMode set_mode;
    set_mode.request.custom_mode = "OFFBOARD";

    //arming_client
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();	

    while(ros::ok()){

		if( (current_state[0].mode != "OFFBOARD" || current_state[1].mode != "OFFBOARD") && (ros::Time::now() - last_request > ros::Duration(3.0)) ){
	        for(int i=0; i < NUM_DRONE ; i++){
		        if( set_mode_client[i].call(set_mode) && set_mode.response.success){
		            ROS_INFO("Camila%d OFFBOARD enabled", i+1);
		        }
		    }
	        last_request = ros::Time::now();
	    } 
		else{
	        if( (!current_state[0].armed || !current_state[1].armed) && (ros::Time::now() - last_request > ros::Duration(3.0)) ){
	            for(int i=0 ; i < NUM_DRONE ; i++){
		            if( arming_client[i].call(arm_cmd) && arm_cmd.response.success){
		                ROS_INFO("Camila%d armed", i+1);
		            }
		        }
	            last_request = ros::Time::now();
	        
	    	}
	    }
	    
	    nh.getParam("control_node/x",com_x);
	    nh.getParam("control_node/y",com_y);
	    nh.getParam("control_node/z",com_z);

	    set_positon(com_x, com_y, com_z);

	    for(int i=0 ; i < NUM_DRONE ; i++){
	    	local_pos_pub[i].publish(l_pos[i]);	
	    }

        ros::spinOnce();
        rate.sleep();
        
    }

    return 0;
}

void set_positon(double x, double y, double z){
	    if(x != coor[0] | y != coor[1] | z != coor[2]){
	    	ROS_INFO("(%lf, %lf, %lf)", com_x, com_y, com_z);	
	    	l_pos[0].pose.position.x = x;
	    	l_pos[0].pose.position.y = y;
	    	l_pos[0].pose.position.z = z;

	    	for(int i=1 ; i<NUM_DRONE ; i++)
				l_pos[i] = l_pos[0];

			l_pos[1].pose.position.x += offset;
	    }
	    coor[0] = x;
	    coor[1] = y;
	    coor[2] = z;

}