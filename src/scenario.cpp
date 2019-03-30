#include <scenario.h>

int Scenario::scen_num_;
std::string Scenario::scen_str_ = "";

Scenario::Scenario(ros::NodeHandle &nh, SwarmVehicle &swarm)
	: nh_global_(nh),
	  swarm_(swarm),
	  formation_(""),
	  swarm_target_local_(tf2::Vector3(0, 0, 0))
{
	nh_global_.getParam("num_drone", num_of_vehicle_);
	scen_hex_.reserve(num_of_vehicle_);
	camila_ = swarm_.getSwarmVehicle();
	offset_ = swarm_.getSwarmOffset();

	if (num_of_vehicle_ != 1)
		angle_ = 2.0 * M_PI / (num_of_vehicle_ - 1);

	swarmServiceInit();
}

Scenario::~Scenario()
{
	release();
}

void Scenario::swarmServiceInit()
{
	goto_vehicle_server_ = nh_global_.advertiseService("goto_vehicle", &Scenario::gotoVehicle, this);
	multi_setpoint_local_server_ = nh_global_.advertiseService("multi_setpoint_local", &Scenario::multiSetpointLocal, this);
	multi_setpoint_global_server_ = nh_global_.advertiseService("multi_setpoint_global", &Scenario::multiSetpointGlobal, this);
}

void Scenario::release()
{
	std::vector<uint8_t>().swap(scen_hex_);
	multi_setpoint_global_server_.shutdown();
	multi_setpoint_local_server_.shutdown();
	goto_vehicle_server_.shutdown();
}

void Scenario::formationGenerator()
{
	double spacing, radius;
	nh_global_.getParamCached("spacing", spacing);
	nh_global_.getParamCached("radius", radius);

	if (formation_ == "POINT")
		point();
	else if (formation_ == "IDLE")
		idle(spacing);
	else if (formation_ == "SCEN1")
		makeCircle(radius);
	else if (formation_ == "SCEN2")
		makeEllipse(radius);
	else if (formation_ == "SCEN3")
		drawStringFont7x5(spacing);
	else if (formation_ == "SCEN4")
		drawStringFont8x8(spacing);
}

void Scenario::point()
{
	geometry_msgs::PoseStamped msg;
	int i = 0;
	for (auto vehicle : *camila_)
	{
		msg.pose.position.x = msg.pose.position.x + (*offset_)[i].getX();
		msg.pose.position.y = msg.pose.position.y + (*offset_)[i].getY();
		msg.pose.position.z = msg.pose.position.z;
		vehicle.setLocalTarget(msg);
		i++;
	}
}

void Scenario::idle(double spacing)
{
	geometry_msgs::PoseStamped msg;
	nh_global_.getParamCached("spacing", spacing);
	int i = 0;
	double x = 0, y = -spacing;
	for (auto vehicle : *camila_)
	{
		if (y < 16)
			y += spacing;
		else
		{
			x += spacing;
			y = 0;
		}
		msg.pose.position.x = msg.pose.position.x + (*offset_)[i].getX() + x;
		msg.pose.position.y = msg.pose.position.y + (*offset_)[i].getY() + y;
		msg.pose.position.z = msg.pose.position.z;
		vehicle.setLocalTarget(msg);
		i++;
	}
}

void Scenario::makeCircle(double radius)
{
	geometry_msgs::PoseStamped msg;
	double m_sec = ros::Time::now().toNSec() / 1000000;
	double x = 0.00006 * m_sec; // 0.06 rad/s
	double angle;
	int i = 0;
	for (auto vehicle : *camila_)
	{
		angle = i * angle_;

		msg.pose.position.x = msg.pose.position.x + (*offset_)[i].getX() + radius * cos(angle + x);
		msg.pose.position.y = msg.pose.position.y + (*offset_)[i].getY() + radius * sin(angle + x);
		msg.pose.position.z = msg.pose.position.z;
		vehicle.setLocalTarget(msg);
		i++;
	}
}

void Scenario::makeEllipse(double radius)
{
	geometry_msgs::PoseStamped msg;
	double m_sec = ros::Time::now().toNSec() / 1000000;
	double x = 0.00006 * m_sec; // 0.06 rad/s
	double angle;
	int i = 0;
	for (auto vehicle : *camila_)
	{
		angle = i * angle_;
		msg.header.stamp = ros::Time::now();
		msg.pose.position.x = msg.pose.position.x + (*offset_)[i].getX() + radius * cos(angle + x);
		msg.pose.position.y = msg.pose.position.y + (*offset_)[i].getY() + radius * sin(angle + x);
		msg.pose.position.z = msg.pose.position.z + (*offset_)[i].getZ() + radius * cos(angle + x) * x;
		vehicle.setLocalTarget(msg);
		i++;
	}
}

void Scenario::drawStringFont7x5(double spacing)
{
	std::string scen_str;
	nh_global_.getParamCached("scen", scen_str);
	std::vector<std::pair<int, int>> scen;
	std::vector<std::pair<int, int>>::iterator iter;
	std::vector<uint8_t>::iterator iter_uint8;
	scen.reserve(num_of_vehicle_);

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

	while (scen.size() > num_of_vehicle_)
		scen.pop_back();

	geometry_msgs::PoseStamped temp;
	int scen_size = scen.size();
	int i = 0, j = 0;
	for (auto vehicle : *camila_)
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
		temp.pose.position.x = swarm_target_local_.getX() + (*offset_)[i].getX() + iter->first * spacing;
		temp.pose.position.y = swarm_target_local_.getY() + (*offset_)[i].getY();
		temp.pose.position.z = swarm_target_local_.getZ() + (*offset_)[i].getZ() + iter->second * spacing;
		vehicle.setScenPos(*iter);
		vehicle.setLocalTarget(temp);
		scen.erase(iter);
		i++;
	}
}

void Scenario::drawStringFont8x8(double spacing)
{
	std::string scen_str;
	nh_global_.getParamCached("scen", scen_str);
	std::vector<std::pair<int, int>> scen;
	std::vector<std::pair<int, int>>::iterator iter;
	std::vector<uint8_t>::iterator iter_uint8;
	scen.reserve(num_of_vehicle_);

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

	while (scen.size() > num_of_vehicle_)
		scen.pop_back();

	geometry_msgs::PoseStamped temp;
	int scen_size = scen.size();
	int i = 0, j = 0;
	for (auto vehicle : *camila_)
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
		temp.pose.position.x = swarm_target_local_.getX() + (*offset_)[i].getX() + iter->first * spacing;
		temp.pose.position.y = swarm_target_local_.getX() + (*offset_)[i].getY();
		temp.pose.position.z = swarm_target_local_.getX() + (*offset_)[i].getZ() + iter->second * spacing;
		vehicle.setScenPos(*iter);
		vehicle.setLocalTarget(temp);
		scen.erase(iter);
		i++;
	}
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

bool Scenario::multiSetpointLocal(swarm_ctrl_pkg::srvMultiSetpointLocal::Request &req,
								  swarm_ctrl_pkg::srvMultiSetpointLocal::Response &res)
{
	swarm_.updateOffset();

	formation_ = req.formation;
	swarm_target_local_.setX(req.x);
	swarm_target_local_.setY(req.y);
	swarm_target_local_.setZ(req.z);

	res.success = true;
	return res.success;
}

bool Scenario::multiSetpointGlobal(swarm_ctrl_pkg::srvMultiSetpointGlobal::Request &req,
								   swarm_ctrl_pkg::srvMultiSetpointGlobal::Response &res)
{
	res.success = true;
	return res.success;
}

bool Scenario::gotoVehicle(swarm_ctrl_pkg::srvGoToVehicle::Request &req,
						   swarm_ctrl_pkg::srvGoToVehicle::Response &res)
{
	geometry_msgs::PoseStamped msg;
	auto vehicle = (*camila_)[req.num_drone - 1];

	swarm_.updateOffset();

	if (req.num_drone > 0 && req.num_drone <= num_of_vehicle_)
	{
		msg.header.stamp = ros::Time::now();
		msg.pose.position.x = req.x + (*offset_)[req.num_drone - 1].getX();
		msg.pose.position.y = req.y + (*offset_)[req.num_drone - 1].getY();
		msg.pose.position.z = req.z;
	}
	vehicle.setLocalTarget(msg);

	return true;
}