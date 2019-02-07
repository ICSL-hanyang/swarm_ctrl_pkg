#include <ros/ros.h>
#include <vehicle.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "swarm_node");
	ros::NodeHandle nh("~");
	ros::Rate rate(10);

	std::unique_ptr<SwarmVehicle> camila(new SwarmVehicle(nh, "camila", 6));

	ROS_INFO("swarm_node start");
	geometry_msgs::PoseStamped msg;
	msg.pose.position.x = 0;
	msg.pose.position.y = 10;
	msg.pose.position.z = 2;

	mavros_msgs::State state;
	while (ros::ok())
	{
		/* state=camila->
		if(state.connected==true && state.armed != true){
			camila0.arming(true);
			camila0.setMode("auto.takeoff");
		}
		else{
			camila0.gotoLocal(msg);
			camila0.setMode("offboard");
		} */

		//camila.camila[0].gotoLocal(msg);
		camila->run();

		ros::spinOnce();
		rate.sleep();
	}

	camila.release();
	return 0;
}