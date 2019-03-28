#include <scenario.h>

Scenario::Scenario(ros::NodeHandle &nh, SwarmVehicle &swarm)
	: nh_global_(nh),
	  swarm_(swarm)
{
	camila_ = swarm_.getSwarmVehicle();
	offset_ = swarm_.getSwarmOffset();
	formation_ = swarm_.getCurrentFormation();
	swarm_target_local_ = swarm_.getSwarmTargetLocal();
}

void Scenario::formationGenerator()
{
	double spacing;
	nh_global_.getParamCached("spacing", spacing);

	if (*formation_ == "POINT")
		point();
	else if (*formation_ == "IDLE")
		idle();
	else if (*formation_ == "SCEN1")
		makeCircle();
	else if (*formation_ == "SCEN2")
		makeEllipse();
	else if (*formation_ == "SCEN3")
		drawStringFont7x5();
	else if (*formation_ == "SCEN4")
		drawStringFont8x8();
}

void Scenario::point()
{
	geometry_msgs::PoseStamped msg, msg_f;
	double m_sec = ros::Time::now().toNSec() / 1000000;
	double x = 0.00006 * m_sec; // 0.2 rad/s
	double angle, spacing;
	int i = 0;
	for (auto &vehicle : *camila_)
	{
		if (vehicle.getInfo().vehicle_id_ != 1)
		{
			msg_f.pose.position.x = msg.pose.position.x + offset_[i].getX();
			msg_f.pose.position.y = msg.pose.position.y + offset_[i].getY();
			msg_f.pose.position.z = msg.pose.position.z;
			vehicle.setLocalTarget(msg_f);
		}
		else
			vehicle.setLocalTarget(msg);
		i++;
	}
}

void Scenario::idle()
{
	geometry_msgs::PoseStamped msg, msg_f;
	double m_sec = ros::Time::now().toNSec() / 1000000;
	double x = 0.00006 * m_sec; // 0.2 rad/s
	double angle, spacing;
	int i = 0;
	int x = 0, y = -3;
	for (auto &vehicle : *camila_)
	{
		if (y < 16)
			y += 3;
		else
		{
			x += 3;
			y = 0;
		}
		msg_f.pose.position.x = msg.pose.position.x + offset_[i].getX() + x;
		msg_f.pose.position.y = msg.pose.position.y + offset_[i].getY() + y;
		msg_f.pose.position.z = msg.pose.position.z;
		vehicle.setLocalTarget(msg_f);
		i++;
	}
}

void Scenario::makeCircle()
{
	geometry_msgs::PoseStamped msg, msg_f;
	double m_sec = ros::Time::now().toNSec() / 1000000;
	double x = 0.00006 * m_sec; // 0.2 rad/s
	double angle, spacing;
	int i = 0;
	for (auto &vehicle : *camila_)
	{
		if (vehicle.getInfo().vehicle_id_ != 1)
		{
			angle = i * angle_;

			msg_f.pose.position.x = msg.pose.position.x + offset_[i].getX() + spacing * cos(angle + x);
			msg_f.pose.position.y = msg.pose.position.y + offset_[i].getY() + spacing * sin(angle + x);
			msg_f.pose.position.z = msg.pose.position.z;
			vehicle.setLocalTarget(msg_f);
		}
		else
			vehicle.setLocalTarget(msg);
		i++;
	}
}

void Scenario::makeEllipse()
{
	geometry_msgs::PoseStamped msg, msg_f;
	double m_sec = ros::Time::now().toNSec() / 1000000;
	double x = 0.00006 * m_sec; // 0.2 rad/s
	double angle, spacing;
	nh_global_.getParamCached("spacing", spacing);

	msg.pose.position.x = swarm_.getSwarmTargetLocal.getX();
	msg.pose.position.y = swarm_.getSwarmTargetLocal.getY();
	msg.pose.position.z = swarm_.getSwarmTargetLocal.getZ();

	int i = 0;
	for (auto &vehicle : *camila_)
	{
		if (vehicle.getInfo().vehicle_id_ != 1)
		{
			angle = i * angle_;
			msg_f.header.stamp = ros::Time::now();
			msg_f.pose.position.x = msg.pose.position.x + offset_[i].getX() + spacing * cos(angle + x);
			msg_f.pose.position.y = msg.pose.position.y + offset_[i].getY() + spacing * sin(angle + x);
			msg_f.pose.position.z = msg.pose.position.z + offset_[i].getZ() + spacing * cos(angle + x) * x;
			vehicle.setLocalTarget(msg_f);
		}
		else
		{
			msg.header.stamp = ros::Time::now();
			vehicle.setLocalTarget(msg);
		}
		i++;
	}
}

void Scenario::drawStringFont7x5()
{
	std::string scen_str;
	int num_of_vehicle;
	double spacing;
	nh_global_.getParamCached("scen", scen_str);
	nh_global_.getParamCached("spacing", spacing);
	nh_global_.getParamCached("num_drone", num_of_vehicle);
	std::vector<std::pair<int, int>> scen;
	std::vector<std::pair<int, int>>::iterator iter;
	std::vector<uint8_t>::iterator iter_uint8;
	scen.reserve(num_of_vehicle);

	if (scen_hex_.size() == 0 || scen_str_ != scen_str)
	{
		scen_hex_.clear();
		scen_str_ = scen_str;
		for (auto &character : scen_str_)
		{
			int scen_num = static_cast<int>(character);
			for (auto &line : FONT2[scen_num])
			{
				scen_hex_.push_back(line);
			}
		}
		prev_ = ros::Time::now();
	}
	else
	{
		if (ros::Time::now() > prev_ + ros::Duration(20.0))
		{
			prev_ = ros::Time::now();
			iter_uint8 = scen_hex_.begin();
			scen_hex_.erase(iter_uint8);
			iter_uint8 = scen_hex_.begin();
			scen_hex_.erase(iter_uint8);
			iter_uint8 = scen_hex_.begin();
			scen_hex_.erase(iter_uint8);
			iter_uint8 = scen_hex_.begin();
			scen_hex_.erase(iter_uint8);
			iter_uint8 = scen_hex_.begin();
			scen_hex_.erase(iter_uint8);
		}
	}
	int t = 0;
	for (auto &hex : scen_hex_)
	{
		uint8_t left_bits, right_bits;
		left_bits = hex >> 4;
		right_bits = 0x0f & hex;

		hexToCoord(scen, left_bits, 5 - t, false);
		hexToCoord(scen, right_bits, 5 - t, true);
		t++;
	}

	while (scen.size() > num_of_vehicle)
		scen.pop_back();

	geometry_msgs::PoseStamped temp;
	int scen_size = scen.size();
	int i = 0, j = 0;
	for (auto &vehicle : *camila_)
	{
		if (scen.size() == 0)
		{
			hexToCoord(scen, 0x0f, -3 - j, true);
			hexToCoord(scen, 0x0f, -3 - j, false);
			j++;
		}
		int min_num = 0;
		int min_dist = 350;
		int k = 0;
		std::pair<int, int> prev_scen = vehicle.getScenPos();
		for (iter = scen.begin(); iter != scen.end(); iter++)
		{
			int a, b, length;
			a = prev_scen.first - iter->first;
			b = prev_scen.second - iter->second;
			length = a * a + b * b;
			if (length < min_dist)
			{
				min_dist = length;
				min_num = k;
			}
			k++;
		}
		iter = scen.begin() + min_num;
		temp.header.stamp = ros::Time::now();
		temp.pose.position.x = swarm_.getSwarmTargetLocal.getX() + offset_[i].getX() + iter->first * spacing;
		temp.pose.position.y = swarm_.getSwarmTargetLocal.getY() + offset_[i].getY();
		temp.pose.position.z = swarm_.getSwarmTargetLocal.getZ() + offset_[i].getZ() + iter->second * spacing;
		vehicle.setScenPos(*iter);
		vehicle.setLocalTarget(temp);
		scen.erase(iter);
		i++;
	}
}

void Scenario::drawStringFont8x8()
{
	std::string scen_str;
	double spacing;
	int num_of_vehicle;
	nh_global_.getParamCached("scen", scen_str);
	nh_global_.getParamCached("spacing", spacing);
	nh_global_.getParamCached("num_drone", num_of_vehicle);
	std::vector<std::pair<int, int>> scen;
	std::vector<std::pair<int, int>>::iterator iter;
	std::vector<uint8_t>::iterator iter_uint8;
	scen.reserve(num_of_vehicle);

	if (scen_hex_.size() == 0 || scen_str_ != scen_str)
	{
		scen_hex_.clear();
		scen_str_ = scen_str;
		for (auto &character : scen_str_)
		{
			int scen_num = static_cast<int>(character);
			for (auto &line : FONT[scen_num])
			{
				scen_hex_.push_back(line);
			}
		}
		prev_ = ros::Time::now();
	}
	else
	{
		if (ros::Time::now() > prev_ + ros::Duration(35.0))
		{
			prev_ = ros::Time::now();
			iter_uint8 = scen_hex_.begin();
			scen_hex_.erase(iter_uint8);
			iter_uint8 = scen_hex_.begin();
			scen_hex_.erase(iter_uint8);
			iter_uint8 = scen_hex_.begin();
			scen_hex_.erase(iter_uint8);
			iter_uint8 = scen_hex_.begin();
			scen_hex_.erase(iter_uint8);
			iter_uint8 = scen_hex_.begin();
			scen_hex_.erase(iter_uint8);
			iter_uint8 = scen_hex_.begin();
			scen_hex_.erase(iter_uint8);
			iter_uint8 = scen_hex_.begin();
			scen_hex_.erase(iter_uint8);
			iter_uint8 = scen_hex_.begin();
			scen_hex_.erase(iter_uint8);
		}
	}

	int t = 0;
	for (auto &hex : scen_hex_)
	{
		uint8_t left_bits, right_bits;
		left_bits = hex >> 4;
		right_bits = 0x0f & hex;

		hexToCoord(scen, left_bits, 8 - t, true);
		hexToCoord(scen, right_bits, 8 - t, false);
		t++;
	}

	while (scen.size() > num_of_vehicle)
		scen.pop_back();

	geometry_msgs::PoseStamped temp;
	int scen_size = scen.size();
	int i = 0, j = 0;
	for (auto &vehicle : camila_)
	{
		if (scen.size() == 0)
		{
			hexToCoord(scen, 0x0f, -3 - j, true);
			hexToCoord(scen, 0x0f, -3 - j, false);
			j++;
		}
		int min_num = 0;
		int min_dist = 350;
		int k = 0;
		std::pair<int, int> prev_scen = vehicle.getScenPos();
		for (iter = scen.begin(); iter != scen.end(); iter++)
		{
			int a, b, length;
			a = prev_scen.first - iter->first;
			b = prev_scen.second - iter->second;
			length = a * a + b * b;
			if (length < min_dist)
			{
				min_dist = length;
				min_num = k;
			}
			k++;
		}
		iter = scen.begin() + min_num;
		temp.header.stamp = ros::Time::now();
		temp.pose.position.x = swarm_.getSwarmTargetLocal.getX() + offset_[i].getX() + iter->first * spacing;
		temp.pose.position.y = swarm_.getSwarmTargetLocal.getY() + offset_[i].getY();
		temp.pose.position.z = swarm_.getSwarmTargetLocal.getZ() + offset_[i].getZ() + iter->second * spacing;
		vehicle.setScenPos(*iter);
		vehicle.setLocalTarget(temp);
		scen.erase(iter);
		i++;
	}
}

geometry_msgs::PoseStamped Scenario::getSwarmTarget() const
{
	tf2::Vector3 target;
	target = swarm_.getSwarmTargetLocal();
	geometry_msgs::PoseStamped msg;
	msg.pose.position.x = target.getX();
	msg.pose.position.y = target.getY();
	msg.pose.position.z = target.getZ();
	return msg;
}

void Scenario::hexToCoord(std::vector<std::pair<int, int>> &scen, const uint8_t &hex, const int &x_value, const bool &is_left_bits)
{
	int offset, a, b, c, d;
	if (is_left_bits)
		offset = 0;
	else
		offset = 4;

	if (formation_ == "SCEN4")
	{
		a = 3;
		b = 2;
		c = 1;
		d = 0;
	}
	else if (formation_ == "SCEN3")
	{
		a = 0;
		b = 1;
		c = 2;
		d = 3;
	}

	switch (hex)
	{
	case 1:
		scen.push_back(std::pair<int, int>(x_value, a + offset));
		break;
	case 2:
		scen.push_back(std::pair<int, int>(x_value, b + offset));
		break;
	case 3:
		scen.push_back(std::pair<int, int>(x_value, b + offset));
		scen.push_back(std::pair<int, int>(x_value, a + offset));
		break;
	case 4:
		scen.push_back(std::pair<int, int>(x_value, c + offset));
		break;
	case 5:
		scen.push_back(std::pair<int, int>(x_value, c + offset));
		scen.push_back(std::pair<int, int>(x_value, a + offset));
		break;
	case 6:
		scen.push_back(std::pair<int, int>(x_value, c + offset));
		scen.push_back(std::pair<int, int>(x_value, b + offset));
		break;
	case 7:
		scen.push_back(std::pair<int, int>(x_value, c + offset));
		scen.push_back(std::pair<int, int>(x_value, b + offset));
		scen.push_back(std::pair<int, int>(x_value, a + offset));
		break;
	case 8:
		scen.push_back(std::pair<int, int>(x_value, d + offset));
		break;
	case 9:
		scen.push_back(std::pair<int, int>(x_value, d + offset));
		scen.push_back(std::pair<int, int>(x_value, a + offset));
		break;
	case 10:
		scen.push_back(std::pair<int, int>(x_value, d + offset));
		scen.push_back(std::pair<int, int>(x_value, b + offset));
		break;
	case 11:
		scen.push_back(std::pair<int, int>(x_value, d + offset));
		scen.push_back(std::pair<int, int>(x_value, b + offset));
		scen.push_back(std::pair<int, int>(x_value, a + offset));
		break;
	case 12:
		scen.push_back(std::pair<int, int>(x_value, d + offset));
		scen.push_back(std::pair<int, int>(x_value, c + offset));
		break;
	case 13:
		scen.push_back(std::pair<int, int>(x_value, d + offset));
		scen.push_back(std::pair<int, int>(x_value, c + offset));
		scen.push_back(std::pair<int, int>(x_value, a + offset));
		break;
	case 14:
		scen.push_back(std::pair<int, int>(x_value, d + offset));
		scen.push_back(std::pair<int, int>(x_value, c + offset));
		scen.push_back(std::pair<int, int>(x_value, b + offset));
		break;
	case 15:
		scen.push_back(std::pair<int, int>(x_value, d + offset));
		scen.push_back(std::pair<int, int>(x_value, c + offset));
		scen.push_back(std::pair<int, int>(x_value, b + offset));
		scen.push_back(std::pair<int, int>(x_value, a + offset));
		break;

	default:
		break;
	}
}