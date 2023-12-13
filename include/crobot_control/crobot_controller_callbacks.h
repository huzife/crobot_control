#ifndef CROBOT_CONTROLLER_CALLBACKS_H
#define CROBOT_CONTROLLER_CALLBACKS_H

#include "ros/ros.h"
#include "com_control/controller_callbacks.h"

namespace crobot_ros {

class Controller_CB: public crobot::Controller_Callbacks {
private:
    ros::NodeHandle nh;
    ros::Publisher velocity_pub;
    ros::Publisher odom_pub;
    ros::Publisher imu_raw_pub;
    ros::Publisher imu_temperature_pub;

    ros::Time current_time;
    ros::Time last_time;

    double current_x;
    double current_y;
    double current_yaw;

public:
    Controller_CB(ros::NodeHandle& nh) : nh(nh) {
        init();
    }

    void set_speed_callback() override;

    void get_speed_callback(const crobot::Get_Speed_Resp& resp) override;

    void get_imu_temperature_callback(const crobot::Get_IMU_Temperature_Resp& resp) override;

    void get_imu_callback(const crobot::Get_IMU_Resp& resp) override;

private:
    void init();
};

} // namespace crobot_ros

#endif // CROBOT_CONTROLLER_CALLBACKS_H
