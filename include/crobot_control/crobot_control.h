#ifndef _CROBOT_CONTROL_H
#define _CROBOT_CONTROL_H

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "com_control/controller.h"
#include "com_control/controller_callbacks.h"
#include <functional>
#include <string>
#include <thread>

namespace crobot_ros {

class CRobotControl {
private:
	ros::NodeHandle nh;
	ros::NodeHandle nh_private;
	ros::Subscriber cmd_vel_sub;

	crobot::Controller controller;

	std::string port_name;
	uint32_t baudrate;
	uint8_t parity;
	uint8_t databit;
	uint8_t stopbit;
	uint8_t flow_control;

    std::thread get_temp_hum_thread;
	std::thread get_speed_thread;

public:
	CRobotControl(ros::NodeHandle nh, ros::NodeHandle nh_private, crobot::ControllerCallbacks &cbs)
		: nh(nh), nh_private(nh_private), controller(cbs) {}

	void init();
	void start();
    
private:
	void twistSubscribeCB(const geometry_msgs::Twist::ConstPtr &msg);
    void getTempAndHumFunc();
	void getSpeedFunc();
};

} // namespace crobot_ros

#endif // _CROBOT_CONTROL_H