#include "crobot_control/crobot_controller_callbacks.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt16.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"

namespace crobot_ros {

Controller_CB::Controller_CB(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private) {
    init();
}

void Controller_CB::set_velocity_callback() {}

void Controller_CB::get_odom_callback(const crobot::Get_Odom_Resp& resp) {
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    // 位姿
    odom.pose.pose.position.x = resp.position_x;
    odom.pose.pose.position.y = resp.position_y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(resp.direction);

    // 速度
    odom.twist.twist.linear.x = resp.linear_x;
    odom.twist.twist.linear.y = resp.linear_y;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = resp.angular_z;

    odom_pub_.publish(odom);
}

void Controller_CB::get_imu_temperature_callback(const crobot::Get_IMU_Temperature_Resp& resp) {
    std_msgs::Float32 temperature;
    temperature.data = resp.temperature;

    imu_temperature_pub_.publish(temperature);
}

void Controller_CB::get_imu_data_callback(const crobot::Get_IMU_Data_Resp& resp) {
    sensor_msgs::Imu imu_raw_data;
    imu_raw_data.header.stamp = ros::Time::now();
    imu_raw_data.header.frame_id = "imu_link";

    imu_raw_data.linear_acceleration.x = resp.accel_x;
    imu_raw_data.linear_acceleration.y = resp.accel_y;
    imu_raw_data.linear_acceleration.z = resp.accel_z;
    imu_raw_data.angular_velocity.x = resp.angular_x;
    imu_raw_data.angular_velocity.y = resp.angular_y;
    imu_raw_data.angular_velocity.z = resp.angular_z;

    imu_raw_data_pub_.publish(imu_raw_data);
}

void Controller_CB::get_ultrasonic_range_callback(const crobot::Get_Ultrasonic_Range_Resp& resp) {
    std_msgs::UInt16 range;
    range.data = resp.range;

    ultrasonic_range_pub_.publish(range);
}

void Controller_CB::get_battery_voltage_callback(const crobot::Get_Battery_Voltage_Resp& resp) {
    std_msgs::Float32 voltage;
    voltage.data = resp.voltage;

    battery_voltage_pub_.publish(voltage);
}

void Controller_CB::init() {
    odom_pub_ = nh_private_.advertise<nav_msgs::Odometry>("odom", 10);
    imu_temperature_pub_ = nh_private_.advertise<std_msgs::Float32>("imu/temperature", 10);
    imu_raw_data_pub_ = nh_private_.advertise<sensor_msgs::Imu>("imu/raw_data", 10);
    ultrasonic_range_pub_ = nh_private_.advertise<std_msgs::UInt16>("ultrasonic/range", 10);
    battery_voltage_pub_ = nh_private_.advertise<std_msgs::Float32>("battery_voltage", 10);
}

} // namespace crobot_ros
