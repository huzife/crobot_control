#ifndef CROBOT_CONTROLLER_CALLBACKS_H
#define CROBOT_CONTROLLER_CALLBACKS_H

#include "ros/ros.h"
#include "com_control/controller_callbacks.h"

namespace crobot_ros {

class Controller_CB: public crobot::Controller_Callbacks {
private:
    ros::NodeHandle nh;
    ros::Publisher odom_pub;
    ros::Publisher imu_raw_pub;
    ros::Publisher imu_temperature_pub;

public:
    Controller_CB(ros::NodeHandle& nh) : nh(nh) {
        init();
    }

    void set_velocity_callback() override;

    void get_odom_callback(const crobot::Get_Odom_Resp& resp) override;

    void get_imu_temperature_callback(const crobot::Get_IMU_Temperature_Resp& resp) override;

    void get_imu_data_callback(const crobot::Get_IMU_Data_Resp& resp) override;

private:
    void init();
};

} // namespace crobot_ros

#endif // CROBOT_CONTROLLER_CALLBACKS_H
