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
	scens_.reserve(num_of_vehicle_);
	scen_hex_.reserve(num_of_vehicle_);

	if (num_of_vehicle_ != 1)
		angle_ = 2.0 * M_PI / (num_of_vehicle_ - 1);

	swarmServiceInit();
}

Scenario::Scenario(const Scenario &rhs)
	: nh_global_(rhs.nh_global_),
	  swarm_(rhs.swarm_),
	  formation_(rhs.formation_),
	  swarm_target_local_(rhs.swarm_target_local_)
{
	num_of_vehicle_ = rhs.num_of_vehicle_;
	scens_.reserve(num_of_vehicle_);
	scen_hex_.reserve(num_of_vehicle_);

	if (num_of_vehicle_ != 1)
		angle_ = 2.0 * M_PI / (num_of_vehicle_ - 1);

	swarmServiceInit();
}

const Scenario &Scenario::operator=(const Scenario &rhs)
{
	if(this == &rhs)
		return *this;
	
	std::vector<tf2::Vector3>().swap(scens_);
	std::vector<uint8_t>().swap(scen_hex_);
	num_of_vehicle_ = rhs.num_of_vehicle_;
	scens_.reserve(num_of_vehicle_);
	scen_hex_.reserve(num_of_vehicle_);

	if (num_of_vehicle_ != 1)
		angle_ = 2.0 * M_PI / (num_of_vehicle_ - 1);

	nh_global_ = rhs.nh_global_;
	swarm_ = rhs.swarm_;
	formation_ = rhs.formation_;
	swarm_target_local_ = rhs.swarm_target_local_;

	swarmServiceInit();
}

Scenario::~Scenario()
{
	release();
}

void Scenario::swarmServiceInit()
{
	multi_setpoint_local_server_ = nh_global_.advertiseService("multi_setpoint_local", &Scenario::multiSetpointLocal, this);
	multi_setpoint_global_server_ = nh_global_.advertiseService("multi_setpoint_global", &Scenario::multiSetpointGlobal, this);
}

void Scenario::release()
{
	std::vector<tf2::Vector3>().swap(scens_);
	std::vector<uint8_t>().swap(scen_hex_);
	multi_setpoint_global_server_.shutdown();
	multi_setpoint_local_server_.shutdown();
}

void Scenario::point()
{
	scens_.clear();
	swarm_.setScenario(swarm_target_local_, scens_);
}

void Scenario::idle(double spacing)
{
	scens_.clear();
	double x = 0, y = -spacing;
	for (int i = 0; i < num_of_vehicle_; i++)
	{
		if (y < 20)
			y += spacing;
		else
		{
			x += spacing;
			y = 0;
		}
		scens_.push_back(tf2::Vector3(x, y, 0));
	}
	swarm_.setScenario(swarm_target_local_, scens_);
}

void Scenario::makeCircle(double radius)
{
	scens_.clear();
	double m_sec = ros::Time::now().toNSec() / 1000000;
	double x = 0.00006 * m_sec; // 0.06 rad/s
	double angle;
	for (int i = 0; i < num_of_vehicle_; i++)
	{
		if(i != 0)
		{
			angle = i * angle_;
			scens_.push_back(tf2::Vector3(radius * cos(angle + x), radius * sin(angle + x), 0));
		}
		else
			scens_.push_back(tf2::Vector3(0,  0, 0));
	}
	swarm_.setScenario(swarm_target_local_, scens_);
}

void Scenario::makeEllipse(double radius)
{
	scens_.clear();
	double m_sec = ros::Time::now().toNSec() / 1000000;
	double x = 0.00006 * m_sec; // 0.06 rad/s
	double angle;
	for (int i = 1; i < num_of_vehicle_; i++)
	{
		angle = i * angle_;
		scens_.push_back(tf2::Vector3(radius * cos(angle + x), radius * sin(angle + x), radius * cos(angle + x) * x));
	}
	swarm_.setScenario(swarm_target_local_, scens_);
}

void Scenario::drawStringFont7x5(double spacing)
{
	std::string scen_str;
	nh_global_.getParamCached("scen", scen_str);
	std::vector<std::pair<int, int>> dots;
	std::vector<std::pair<int, int>>::iterator iter;
	std::vector<uint8_t>::iterator iter_uint8;
	dots.reserve(num_of_vehicle_);

	if (scen_hex_.empty() || scen_str_ != scen_str)
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

		hexToCoord(dots, left_bits, 5 - t, false);
		hexToCoord(dots, right_bits, 5 - t, true);
		t++;
	}

	while (dots.size() > num_of_vehicle_)
		dots.pop_back();

	scens_.clear();
	int scen_size = dots.size();
	int j = 0;
	for (int i = 0; i < num_of_vehicle_; i++)
	{
		if (dots.empty())
		{
			hexToCoord(dots, 0x0f, -3 - j, true);
			hexToCoord(dots, 0x0f, -3 - j, false);
			j++;
		}
		// int min_num = 0;
		// int min_dist = 350;
		// int k = 0;
		// std::pair<int, int> prev_scen = vehicle.getScenPos();
		// for (iter = dots.begin(); iter != dots.end(); iter++)
		// {
		// 	int a, b, length;
		// 	a = prev_scen.first - iter->first;
		// 	b = prev_scen.second - iter->second;
		// 	length = a * a + b * b;
		// 	if (length < min_dist)
		// 	{
		// 		min_dist = length;
		// 		min_num = k;
		// 	}
		// 	k++;
		// }
		// iter = dots.begin() + min_num;

		scens_.push_back(tf2::Vector3(iter->first * spacing, 0, iter->second * spacing));

		// vehicle.setScenPos(*iter);
		// dots.erase(iter);
		i++;
	}
	swarm_.setScenario(swarm_target_local_, scens_);
}

void Scenario::drawStringFont8x8(double spacing)
{
	std::string scen_str;
	nh_global_.getParamCached("scen", scen_str);
	std::vector<std::pair<int, int>> dots;
	std::vector<std::pair<int, int>>::iterator iter;
	std::vector<uint8_t>::iterator iter_uint8;
	dots.reserve(num_of_vehicle_);

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

		hexToCoord(dots, left_bits, 8 - t, true);
		hexToCoord(dots, right_bits, 8 - t, false);
		t++;
	}

	while (dots.size() > num_of_vehicle_)
		dots.pop_back();

	scens_.clear();
	int scen_size = dots.size();
	int j = 0;
	for (int i = 0; i < num_of_vehicle_; i++)
	{
		if (dots.empty())
		{
			hexToCoord(dots, 0x0f, -3 - j, true);
			hexToCoord(dots, 0x0f, -3 - j, false);
			j++;
		}
		// int min_num = 0;
		// int min_dist = 350;
		// int k = 0;
		// std::pair<int, int> prev_scen = vehicle.getScenPos();
		// for (iter = dots.begin(); iter != dots.end(); iter++)
		// {
		// 	int a, b, length;
		// 	a = prev_scen.first - iter->first;
		// 	b = prev_scen.second - iter->second;
		// 	length = a * a + b * b;
		// 	if (length < min_dist)
		// 	{
		// 		min_dist = length;
		// 		min_num = k;
		// 	}
		// 	k++;
		// }
		// iter = dots.begin() + min_num;

		scens_.push_back(tf2::Vector3(iter->first * spacing, 0, iter->second * spacing));

		// vehicle.setScenPos(*iter);
		dots.erase(iter);
		i++;
	}
	swarm_.setScenario(swarm_target_local_, scens_);
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