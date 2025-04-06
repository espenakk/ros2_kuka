#include "MBServer.hpp"
#include "rclcpp/rclcpp.hpp"

class Slave : public rclcpp::Node
{
public:
	Slave();

	void update();

private:
	MBServer m_server;
	int m_programIndex;
	float m_param1;
	int m_param2;
	int m_param3;
	int m_param4;
};
