#ifndef CROBOT_CONTROL_H
#define CROBOT_CONTROL_H

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "com_control/controller.h"
#include "com_control/controller_callbacks.h"
#include <string>
#include <thread>

namespace crobot_ros {

class CRobot_Control {
private:
    ros::NodeHandle nh;
    ros::NodeHandle nh_private;
    ros::Subscriber cmd_vel_sub;

    crobot::Controller controller;

    bool thread_end = false;
    std::thread get_odom_thread;
    std::thread get_imu_temperature_thread;
    std::thread get_imu_data_thread;

public:
    CRobot_Control(ros::NodeHandle nh,
                   ros::NodeHandle nh_private,
                   crobot::Controller_Callbacks& cbs);
    ~CRobot_Control();

    void init();
    void start();

private:
    void twist_subscribe_CB(const geometry_msgs::Twist::ConstPtr& msg);
    void get_odom_func();
    void get_imu_temperature_func();
    void get_imu_data_func();
};

} // namespace crobot_ros

#endif // CROBOT_CONTROL_H
