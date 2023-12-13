#ifndef CROBOT_CONTROL_H
#define CROBOT_CONTROL_H

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "com_control/controller.h"
#include "com_control/controller_callbacks.h"
#include <functional>
#include <string>
#include <thread>

namespace crobot_ros {

class CRobot_Control {
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

public:
    CRobot_Control(ros::NodeHandle nh, ros::NodeHandle nh_private, crobot::Controller_Callbacks& cbs)
        : nh(nh), nh_private(nh_private), controller(cbs) {}

    void init();
    void start();

private:
    void twist_subscribe_CB(const geometry_msgs::Twist::ConstPtr& msg);
    void get_speed_func();
    void get_imu_temperature_func();
    void get_imu_func();
};

} // namespace crobot_ros

#endif // CROBOT_CONTROL_H
