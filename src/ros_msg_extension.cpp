#include <ros_msg_extension.h>

const sensor_msgs::NavSatFix operator+(const sensor_msgs::NavSatFix &a, const sensor_msgs::NavSatFix &b)
{
	sensor_msgs::NavSatFix result;
	result.latitude = a.latitude + b.latitude;
	result.longitude = a.longitude + b.longitude;
	result.altitude = a.altitude + b.altitude;

	return result;
}

sensor_msgs::NavSatFix &operator+=(sensor_msgs::NavSatFix &a, const sensor_msgs::NavSatFix &b)
{
	a.latitude += b.latitude;
	a.longitude += b.longitude;
	a.altitude += b.altitude;

	return a;
}

void transformSender(double x, double y, double z, double roll, double pitch, double yaw, ros::Time call_time, const std::string &frame_id, const std::string &child_frame_id)
{
	tf2_ros::TransformBroadcaster tf_br;
	geometry_msgs::TransformStamped transformStamped;
	transformStamped.header.stamp = call_time;
	transformStamped.header.frame_id = frame_id;
	transformStamped.child_frame_id = child_frame_id;
	transformStamped.transform.translation.x = x;
	transformStamped.transform.translation.y = y;
	transformStamped.transform.translation.z = z;
	tf2::Quaternion q;
	q.setRPY(roll, pitch, yaw);
	transformStamped.transform.rotation.x = q.x();
	transformStamped.transform.rotation.y = q.y();
	transformStamped.transform.rotation.z = q.z();
	transformStamped.transform.rotation.w = q.w();
	tf_br.sendTransform(transformStamped);
}
