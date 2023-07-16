#include "crobot_control/crobot_control.h"

namespace crobot_ros {

void CRobotControl::init() {
	if (!nh_private.getParam("port_name", port_name))
		port_name = "/dev/ttyUSB0";

	cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 10,
													 std::bind(&CRobotControl::twistSubscribeCB, this, std::placeholders::_1));

	controller.init(port_name.c_str(),
					itas109::BaudRate115200,
					itas109::ParityNone,
					itas109::DataBits8,
					itas109::StopOne,
					itas109::FlowNone,
					1024);

	get_speed_thread = std::thread(std::bind(&CRobotControl::getSpeedFunc, this));
	get_temp_hum_thread = std::thread(std::bind(&CRobotControl::getTempAndHumFunc, this));
}

void CRobotControl::start() {
    init();
	controller.open();

    get_speed_thread.detach();
	get_temp_hum_thread.detach();
}

void CRobotControl::twistSubscribeCB(const geometry_msgs::Twist::ConstPtr &msg) {
	// ROS_INFO("receive twist");
	crobot::SetSpeedReq speed_req;
	speed_req.linear_x = msg->linear.x;
	speed_req.linear_y = msg->linear.y;
	speed_req.angular_z = msg->angular.z;

	controller.setSpeed(speed_req);
}

void CRobotControl::getSpeedFunc() {
    ros::Rate rate(20);
    while (true) {
        rate.sleep();
        controller.getSpeed();
    }
}

void CRobotControl::getTempAndHumFunc() {
    ros::Rate rate(1);
    while (true) {
        rate.sleep();
        controller.getTempAndHum();
    }
}

} // namespace crobot_ros