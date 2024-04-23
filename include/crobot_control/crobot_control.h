#ifndef CROBOT_CONTROL_H
#define CROBOT_CONTROL_H

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "crobot/controller.h"
#include "crobot/controller_callbacks.h"
#include <string>
#include <thread>

namespace crobot_ros {

class Crobot_Control {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Subscriber cmd_vel_sub_;

    crobot::Controller controller_;

    bool thread_end_ = false;
    std::thread get_odometry_thread_;
    std::thread get_imu_temperature_thread_;
    std::thread get_imu_data_thread_;
    std::thread get_ultrasonic_range_thread_;
    std::thread get_battery_voltage_thread_;

public:
    Crobot_Control(ros::NodeHandle nh,
                   ros::NodeHandle nh_private,
                   crobot::Controller_Callbacks& cbs);
    ~Crobot_Control();

    void init();
    bool start();

private:
    void twist_subscribe_CB(const geometry_msgs::Twist::ConstPtr& msg);
    void get_odometry_func();
    void get_imu_temperature_func();
    void get_imu_data_func();
    void get_ultrasonic_range_func();
    void get_battery_voltage_func();
};

} // namespace crobot_ros

#endif // CROBOT_CONTROL_H
