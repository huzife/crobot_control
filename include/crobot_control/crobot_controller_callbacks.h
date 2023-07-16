#ifndef _CROBOT_CONTROLLER_CALLBACKS_H
#define _CROBOT_CONTROLLER_CALLBACKS_H

#include "ros/ros.h"
#include "com_control/controller_callbacks.h"

namespace crobot_ros {

class ControllerCB : public crobot::ControllerCallbacks {
private:
	ros::NodeHandle nh;
	ros::Publisher temp_hum_pub;
	ros::Publisher velocity_pub;
	ros::Publisher odom_pub;

	ros::Time current_time;
	ros::Time last_time;

	double current_x;
	double current_y;
	double current_yaw;

public:
	ControllerCB(ros::NodeHandle &nh) : nh(nh) {
		init();
	}

	void setSpeedCallback() override;

	void getSpeedCallback(const crobot::GetSpeedResp &resp) override;

	void getTempAndHumCallback(const crobot::GetTempAndHumResp &resp) override;

private:
	void init();
};

} // namespace crobot_ros

#endif // _CROBOT_CONTROLLER_CALLBACKS_H